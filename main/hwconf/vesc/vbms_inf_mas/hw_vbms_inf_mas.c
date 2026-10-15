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

#include "hw_vbms_inf_mas.h"
#include "lispif.h"

typedef struct {
	lbm_uint temp_num;
	lbm_uint batt_ah;
	lbm_uint max_bal_ch;
	lbm_uint soc_use_ah;
	lbm_uint block_sleep;
	lbm_uint vc_empty;
	lbm_uint vc_full;
	lbm_uint vc_balance_start;
	lbm_uint vc_balance_end;
	lbm_uint vc_charge_start;
	lbm_uint vc_charge_end;
	lbm_uint vc_charge_min;
	lbm_uint vc_balance_min;
	lbm_uint balance_max_current;
	lbm_uint min_current_ah_wh_cnt;
	lbm_uint min_current_sleep;
	lbm_uint v_charge_detect;
	lbm_uint t_charge_max;
	lbm_uint t_charge_max_mos;
	lbm_uint sleep_regular;
	lbm_uint sleep_long;
	lbm_uint min_charge_current;
	lbm_uint max_charge_current;
	lbm_uint soc_filter_const;
	lbm_uint t_bal_max_cell;
	lbm_uint t_bal_max_ic;
	lbm_uint t_charge_min;
	lbm_uint t_charge_mon_en;
	lbm_uint psw_t_pchg;
	lbm_uint psw_scd_en;
	lbm_uint psw_scd_tres;
	lbm_uint t_psw_en;
	lbm_uint t_psw_max_mos;
	lbm_uint psw_wait_init;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.temp_num) {
			lbm_add_symbol_const("temp_num", comp);
		} else if (comp == &syms_vesc.batt_ah) {
			lbm_add_symbol_const("batt_ah", comp);
		} else if (comp == &syms_vesc.max_bal_ch) {
			lbm_add_symbol_const("max_bal_ch", comp);
		} else if (comp == &syms_vesc.soc_use_ah) {
			lbm_add_symbol_const("soc_use_ah", comp);
		} else if (comp == &syms_vesc.block_sleep) {
			lbm_add_symbol_const("block_sleep", comp);
		} else if (comp == &syms_vesc.vc_empty) {
			lbm_add_symbol_const("vc_empty", comp);
		} else if (comp == &syms_vesc.vc_full) {
			lbm_add_symbol_const("vc_full", comp);
		} else if (comp == &syms_vesc.vc_balance_start) {
			lbm_add_symbol_const("vc_balance_start", comp);
		} else if (comp == &syms_vesc.vc_balance_end) {
			lbm_add_symbol_const("vc_balance_end", comp);
		} else if (comp == &syms_vesc.vc_charge_start) {
			lbm_add_symbol_const("vc_charge_start", comp);
		} else if (comp == &syms_vesc.vc_charge_end) {
			lbm_add_symbol_const("vc_charge_end", comp);
		} else if (comp == &syms_vesc.vc_charge_min) {
			lbm_add_symbol_const("vc_charge_min", comp);
		} else if (comp == &syms_vesc.vc_balance_min) {
			lbm_add_symbol_const("vc_balance_min", comp);
		} else if (comp == &syms_vesc.balance_max_current) {
			lbm_add_symbol_const("balance_max_current", comp);
		} else if (comp == &syms_vesc.min_current_ah_wh_cnt) {
			lbm_add_symbol_const("min_current_ah_wh_cnt", comp);
		} else if (comp == &syms_vesc.min_current_sleep) {
			lbm_add_symbol_const("min_current_sleep", comp);
		} else if (comp == &syms_vesc.v_charge_detect) {
			lbm_add_symbol_const("v_charge_detect", comp);
		} else if (comp == &syms_vesc.t_charge_max) {
			lbm_add_symbol_const("t_charge_max", comp);
		} else if (comp == &syms_vesc.t_charge_max_mos) {
			lbm_add_symbol_const("t_charge_max_mos", comp);
		} else if (comp == &syms_vesc.sleep_regular) {
			lbm_add_symbol_const("sleep_regular", comp);
		} else if (comp == &syms_vesc.sleep_long) {
			lbm_add_symbol_const("sleep_long", comp);
		} else if (comp == &syms_vesc.min_charge_current) {
			lbm_add_symbol_const("min_charge_current", comp);
		} else if (comp == &syms_vesc.max_charge_current) {
			lbm_add_symbol_const("max_charge_current", comp);
		} else if (comp == &syms_vesc.soc_filter_const) {
			lbm_add_symbol_const("soc_filter_const", comp);
		} else if (comp == &syms_vesc.t_bal_max_cell) {
			lbm_add_symbol_const("t_bal_max_cell", comp);
		} else if (comp == &syms_vesc.t_bal_max_ic) {
			lbm_add_symbol_const("t_bal_max_ic", comp);
		} else if (comp == &syms_vesc.t_charge_min) {
			lbm_add_symbol_const("t_charge_min", comp);
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			lbm_add_symbol_const("t_charge_mon_en", comp);
		} else if (comp == &syms_vesc.t_charge_mon_en) {
			lbm_add_symbol_const("t_charge_mon_en", comp);
		} else if (comp == &syms_vesc.psw_t_pchg) {
			lbm_add_symbol_const("psw_t_pchg", comp);
		} else if (comp == &syms_vesc.psw_scd_en) {
			lbm_add_symbol_const("psw_scd_en", comp);
		} else if (comp == &syms_vesc.psw_scd_tres) {
			lbm_add_symbol_const("psw_scd_tres", comp);
		} else if (comp == &syms_vesc.t_psw_en) {
			lbm_add_symbol_const("t_psw_en", comp);
		} else if (comp == &syms_vesc.t_psw_max_mos) {
			lbm_add_symbol_const("t_psw_max_mos", comp);
		} else if (comp == &syms_vesc.psw_wait_init) {
			lbm_add_symbol_const("psw_wait_init", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_float(bool set, float *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_float(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_float(*val);
	}
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_bool(bool set, bool *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value bms_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg)) {
			lbm_set_error_reason((char *)lbm_error_str_no_number);
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name      = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t *)&backup.config;

	if (compare_symbol(name, &syms_vesc.temp_num)) {
		res = get_or_set_i(set, &cfg->temp_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.batt_ah)) {
		res = get_or_set_float(set, &cfg->batt_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_bal_ch)) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_use_ah)) {
		res = get_or_set_bool(set, &cfg->soc_use_ah, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.block_sleep)) {
		res = get_or_set_bool(set, &cfg->block_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_empty)) {
		res = get_or_set_float(set, &cfg->vc_empty, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_full)) {
		res = get_or_set_float(set, &cfg->vc_full, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_start)) {
		res = get_or_set_float(set, &cfg->vc_balance_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_end)) {
		res = get_or_set_float(set, &cfg->vc_balance_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_start)) {
		res = get_or_set_float(set, &cfg->vc_charge_start, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_end)) {
		res = get_or_set_float(set, &cfg->vc_charge_end, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_charge_min)) {
		res = get_or_set_float(set, &cfg->vc_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.vc_balance_min)) {
		res = get_or_set_float(set, &cfg->vc_balance_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.balance_max_current)) {
		res = get_or_set_float(set, &cfg->balance_max_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_ah_wh_cnt)) {
		res = get_or_set_float(set, &cfg->min_current_ah_wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_current_sleep)) {
		res = get_or_set_float(set, &cfg->min_current_sleep, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_charge_detect)) {
		res = get_or_set_float(set, &cfg->v_charge_detect, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max)) {
		res = get_or_set_float(set, &cfg->t_charge_max, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_max_mos)) {
		res = get_or_set_float(set, &cfg->t_charge_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep_regular)) {
		res = get_or_set_float(set, &cfg->sleep_regular, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.sleep_long)) {
		res = get_or_set_float(set, &cfg->sleep_long, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.min_charge_current)) {
		res = get_or_set_float(set, &cfg->min_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.max_charge_current)) {
		res = get_or_set_float(set, &cfg->max_charge_current, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc_filter_const)) {
		res = get_or_set_float(set, &cfg->soc_filter_const, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_cell)) {
		res = get_or_set_float(set, &cfg->t_bal_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_bal_max_ic)) {
		res = get_or_set_float(set, &cfg->t_bal_max_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_min)) {
		res = get_or_set_float(set, &cfg->t_charge_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_charge_mon_en)) {
		res = get_or_set_bool(set, &cfg->t_charge_mon_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_t_pchg)) {
		res = get_or_set_float(set, &cfg->psw_t_pchg, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_en)) {
		res = get_or_set_bool(set, &cfg->psw_scd_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_scd_tres)) {
		res = get_or_set_i(set, &cfg->psw_scd_tres, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_en)) {
		res = get_or_set_bool(set, &cfg->t_psw_en, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.t_psw_max_mos)) {
		res = get_or_set_float(set, &cfg->t_psw_max_mos, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.psw_wait_init)) {
		res = get_or_set_bool(set, &cfg->psw_wait_init, &set_arg);
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

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Configuration
	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);
}

void hw_init(void) {
	lispif_add_ext_load_callback(load_extensions);
}
