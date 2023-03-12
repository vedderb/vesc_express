/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Joel Svensson    svenssonjoel@yahoo.se

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

#include "main.h"
#include "lispif.h"
#include "lispbm.h"
#include "extensions/array_extensions.h"
#include "extensions/string_extensions.h"
#include "extensions/math_extensions.h"
#include "lispif_disp_extensions.h"
#include "lbm_constants.h"

#include "commands.h"
#include "comm_can.h"
#include "conf_general.h"
#include "mempools.h"
#include "log.h"
#include "buffer.h"
#include "utils.h"
#include "rb.h"
#include "crc.h"
#include "bms.h"
#include "nmea.h"
#include "log_comm.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/i2c.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#include <math.h>
#include <ctype.h>
#include <stdarg.h>

static void(*ext_callback)(void) = 0;
static char print_val_buffer[256];

typedef struct {
	// BMS
	lbm_uint v_tot;
	lbm_uint v_charge;
	lbm_uint i_in;
	lbm_uint i_in_ic;
	lbm_uint ah_cnt;
	lbm_uint wh_cnt;
	lbm_uint cell_num;
	lbm_uint v_cell;
	lbm_uint bal_state;
	lbm_uint temp_adc_num;
	lbm_uint temps_adc;
	lbm_uint temp_ic;
	lbm_uint temp_hum;
	lbm_uint hum;
	lbm_uint temp_max_cell;
	lbm_uint soc;
	lbm_uint soh;
	lbm_uint can_id;
	lbm_uint ah_cnt_chg_total;
	lbm_uint wh_cnt_chg_total;
	lbm_uint ah_cnt_dis_total;
	lbm_uint wh_cnt_dis_total;
	lbm_uint msg_age;

	// GPIO
	lbm_uint pin_mode_out;
	lbm_uint pin_mode_od;
	lbm_uint pin_mode_od_pu;
	lbm_uint pin_mode_od_pd;
	lbm_uint pin_mode_in;
	lbm_uint pin_mode_in_pu;
	lbm_uint pin_mode_in_pd;
	lbm_uint pin_mode_analog;

	// Sysinfo
	lbm_uint hw_name;
	lbm_uint fw_ver;
	lbm_uint uuid;
	lbm_uint hw_type;

	// Rates
	lbm_uint rate_100k;
	lbm_uint rate_200k;
	lbm_uint rate_400k;
	lbm_uint rate_700k;

	// Other
	lbm_uint half_duplex;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool get_add_symbol(char *name, lbm_uint* id) {
	if (!lbm_get_symbol_by_name(name, id)) {
		if (!lbm_add_symbol_const(name, id)) {
			return false;
		}
	}

	return true;
}

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.v_tot) {
			get_add_symbol("bms-v-tot", comp);
		} else if (comp == &syms_vesc.v_charge) {
			get_add_symbol("bms-v-charge", comp);
		} else if (comp == &syms_vesc.i_in) {
			get_add_symbol("bms-i-in", comp);
		} else if (comp == &syms_vesc.i_in_ic) {
			get_add_symbol("bms-i-in-ic", comp);
		} else if (comp == &syms_vesc.ah_cnt) {
			get_add_symbol("bms-ah-cnt", comp);
		} else if (comp == &syms_vesc.wh_cnt) {
			get_add_symbol("bms-wh-cnt", comp);
		} else if (comp == &syms_vesc.cell_num) {
			get_add_symbol("bms-cell-num", comp);
		} else if (comp == &syms_vesc.v_cell) {
			get_add_symbol("bms-v-cell", comp);
		} else if (comp == &syms_vesc.bal_state) {
			get_add_symbol("bms-bal-state", comp);
		} else if (comp == &syms_vesc.temp_adc_num) {
			get_add_symbol("bms-temp-adc-num", comp);
		} else if (comp == &syms_vesc.temps_adc) {
			get_add_symbol("bms-temps-adc", comp);
		} else if (comp == &syms_vesc.temp_ic) {
			get_add_symbol("bms-temp-ic", comp);
		} else if (comp == &syms_vesc.temp_hum) {
			get_add_symbol("bms-temp-hum", comp);
		} else if (comp == &syms_vesc.hum) {
			get_add_symbol("bms-hum", comp);
		} else if (comp == &syms_vesc.temp_max_cell) {
			get_add_symbol("bms-temp-cell-max", comp);
		} else if (comp == &syms_vesc.soc) {
			get_add_symbol("bms-soc", comp);
		} else if (comp == &syms_vesc.soh) {
			get_add_symbol("bms-soh", comp);
		} else if (comp == &syms_vesc.can_id) {
			get_add_symbol("bms-can-id", comp);
		} else if (comp == &syms_vesc.ah_cnt_chg_total) {
			get_add_symbol("bms-ah-cnt-chg-total", comp);
		} else if (comp == &syms_vesc.wh_cnt_chg_total) {
			get_add_symbol("bms-wh-cnt-chg-total", comp);
		} else if (comp == &syms_vesc.ah_cnt_dis_total) {
			get_add_symbol("bms-ah-cnt-dis-total", comp);
		} else if (comp == &syms_vesc.wh_cnt_dis_total) {
			get_add_symbol("bms-wh-cnt-dis-total", comp);
		} else if (comp == &syms_vesc.msg_age) {
			get_add_symbol("bms-msg-age", comp);
		}

		else if (comp == &syms_vesc.pin_mode_out) {
			get_add_symbol("pin-mode-out", comp);
		} else if (comp == &syms_vesc.pin_mode_od) {
			get_add_symbol("pin-mode-od", comp);
		} else if (comp == &syms_vesc.pin_mode_od_pu) {
			get_add_symbol("pin-mode-od-pu", comp);
		} else if (comp == &syms_vesc.pin_mode_od_pd) {
			get_add_symbol("pin-mode-od-pd", comp);
		} else if (comp == &syms_vesc.pin_mode_in) {
			get_add_symbol("pin-mode-in", comp);
		} else if (comp == &syms_vesc.pin_mode_in_pu) {
			get_add_symbol("pin-mode-in-pu", comp);
		} else if (comp == &syms_vesc.pin_mode_in_pd) {
			get_add_symbol("pin-mode-in-pd", comp);
		} else if (comp == &syms_vesc.pin_mode_analog) {
			get_add_symbol("pin-mode-analog", comp);
		}

		else if (comp == &syms_vesc.hw_name) {
			get_add_symbol("hw-name", comp);
		} else if (comp == &syms_vesc.fw_ver) {
			get_add_symbol("fw-ver", comp);
		} else if (comp == &syms_vesc.uuid) {
			get_add_symbol("uuid", comp);
		} else if (comp == &syms_vesc.hw_type) {
			get_add_symbol("hw-type", comp);
		}

		else if (comp == &syms_vesc.rate_100k) {
			get_add_symbol("rate-100k", comp);
		} else if (comp == &syms_vesc.rate_200k) {
			get_add_symbol("rate-200k", comp);
		} else if (comp == &syms_vesc.rate_400k) {
			get_add_symbol("rate-400k", comp);
		} else if (comp == &syms_vesc.rate_700k) {
			get_add_symbol("rate-700k", comp);
		}

		else if (comp == &syms_vesc.half_duplex) {
			get_add_symbol("half-duplex", comp);
		}
	}

	return *comp == sym;
}

static bool is_symbol_true_false(lbm_value v) {
	bool res = lbm_is_symbol_true(v) || lbm_is_symbol_nil(v);
	lbm_set_error_reason("Argument must be t or nil (true or false)");
	return res;
}

// Various commands

static lbm_value ext_print(lbm_value *args, lbm_uint argn) {
	for (lbm_uint i = 0; i < argn; i ++) {
		lbm_value t = args[i];

		if (lbm_is_ptr(t) && lbm_type_of(t) == LBM_TYPE_ARRAY) {
			lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(t);
			switch (array->elt_type){
			case LBM_TYPE_CHAR:
				commands_printf_lisp("%s", (char*)array->data);
				break;
			default:
				return ENC_SYM_NIL;
				break;
			}
		} else if (lbm_type_of(t) == LBM_TYPE_CHAR) {
			if (lbm_dec_char(t) =='\n') {
				commands_printf_lisp(" ");
			} else {
				commands_printf_lisp("%c", lbm_dec_char(t));
			}
		}  else {
			lbm_print_value(print_val_buffer, 256, t);
			commands_printf_lisp("%s", print_val_buffer);
		}
	}

	return ENC_SYM_TRUE;
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

static lbm_value get_set_bms_val(bool set, lbm_value *args, lbm_uint argn) {
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

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name = lbm_dec_sym(args[0]);
	bms_values *val = (bms_values*)bms_get_values();

	if (compare_symbol(name, &syms_vesc.v_tot)) {
		res = get_or_set_float(set, &val->v_tot, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_charge)) {
		res = get_or_set_float(set, &val->v_charge, &res);
	} else if (compare_symbol(name, &syms_vesc.i_in)) {
		res = get_or_set_float(set, &val->i_in, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.i_in_ic)) {
		res = get_or_set_float(set, &val->i_in_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.ah_cnt)) {
		res = get_or_set_float(set, &val->ah_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.wh_cnt)) {
		res = get_or_set_float(set, &val->wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cell_num)) {
		res = get_or_set_i(set, &val->cell_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_cell)) {
		if (argn != 2 || !lbm_is_number(args[1])) {
			return ENC_SYM_EERROR;
		}

		int c = lbm_dec_as_i32(args[1]);
		if (c < 0 || c >= val->cell_num) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_float(set, &val->v_cell[c], &set_arg);
	} else if (compare_symbol(name, &syms_vesc.bal_state)) {
		if (argn != 2 || !lbm_is_number(args[1])) {
			return ENC_SYM_EERROR;
		}

		int c = lbm_dec_as_i32(args[1]);
		if (c < 0 || c >= val->cell_num) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_bool(set, &val->bal_state[c], &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_adc_num)) {
		res = get_or_set_i(set, &val->temp_adc_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temps_adc)) {
		if (argn != 2 || !lbm_is_number(args[1])) {
			return ENC_SYM_EERROR;
		}

		int c = lbm_dec_as_i32(args[1]);
		if (c < 0 || c >= val->temp_adc_num) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_float(set, &val->temps_adc[c], &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_ic)) {
		res = get_or_set_float(set, &val->temp_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_hum)) {
		res = get_or_set_float(set, &val->temp_hum, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.hum)) {
		res = get_or_set_float(set, &val->hum, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_max_cell)) {
		res = get_or_set_float(set, &val->temp_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soc)) {
		res = get_or_set_float(set, &val->soc, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.soh)) {
		res = get_or_set_float(set, &val->soh, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.can_id)) {
		res = get_or_set_i(set, &val->can_id, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.ah_cnt_chg_total)) {
		res = get_or_set_float(set, &val->ah_cnt_chg_total, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.wh_cnt_chg_total)) {
		res = get_or_set_float(set, &val->wh_cnt_chg_total, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.ah_cnt_dis_total)) {
		res = get_or_set_float(set, &val->ah_cnt_dis_total, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.wh_cnt_dis_total)) {
		res = get_or_set_float(set, &val->wh_cnt_dis_total, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.msg_age)) {
		res = lbm_enc_float(UTILS_AGE_S(val->update_time));
	}

	if (res != ENC_SYM_EERROR && set) {
		val->update_time = xTaskGetTickCount();
	}

	return res;
}

static lbm_value ext_get_bms_val(lbm_value *args, lbm_uint argn) {
	return get_set_bms_val(false, args, argn);
}

static lbm_value ext_set_bms_val(lbm_value *args, lbm_uint argn) {
	return get_set_bms_val(true, args, argn);
}

static lbm_value ext_send_bms_can(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	bms_send_status_can();
	return ENC_SYM_TRUE;
}

static lbm_value ext_get_adc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

#ifdef HW_HAS_ADC
	if (argn == 0) {
		return lbm_enc_float(adc_get_voltage(HW_ADC_CH0));
	} else if (argn == 1) {
		lbm_int channel = lbm_dec_as_i32(args[0]);
		if (channel == 0) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH0));
		}

#ifdef HW_ADC_CH1
		else if (channel == 1) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH1));
		}
#endif

#ifdef HW_ADC_CH2
		else if (channel == 2) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH2));
		}
#endif

#ifdef HW_ADC_CH3
		else if (channel == 3) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH3));
		}
#endif

		else {
			return ENC_SYM_EERROR;
		}
	} else {
		return ENC_SYM_EERROR;
	}
#else
	return ENC_SYM_EERROR;
#endif
}

static lbm_value ext_systime(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_u32(xTaskGetTickCount());
}

static lbm_value ext_secs_since(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	return lbm_enc_float(UTILS_AGE_S(lbm_dec_as_u32(args[0])));
}

static lbm_value ext_send_data(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || (!lbm_is_cons(args[0]) && !lbm_is_array(args[0]))) {
		return ENC_SYM_EERROR;
	}

	lbm_value curr = args[0];
	const int max_len = 50;
	uint8_t to_send[max_len];
	uint8_t *to_send_ptr = to_send;
	int ind = 0;

	if (lbm_type_of(args[0]) == LBM_TYPE_ARRAY) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
		if (array->elt_type != LBM_TYPE_BYTE) {
			return ENC_SYM_EERROR;
		}

		to_send_ptr = (uint8_t*)array->data;
		ind = array->size;
	} else {
		while (lbm_is_cons(curr)) {
			lbm_value  arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[ind++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (ind == max_len) {
				break;
			}

			curr = lbm_cdr(curr);
		}
	}

	commands_send_app_data(to_send_ptr, ind);

	return ENC_SYM_TRUE;
}

typedef union {
	uint32_t as_u32;
	int32_t as_i32;
	float as_float;
} eeprom_var;

static bool check_eeprom_addr(int addr) {
	if (addr < 0 || addr > 127) {
		lbm_set_error_reason("Address must be 0 to 127");
		return false;
	}

	return true;
}

static bool store_eeprom_var(eeprom_var *v, int address) {
	if (address < 0 || address > 127) {
		return false;
	}

	char buf[10];
	sprintf(buf, "v%d", address);

	nvs_handle_t my_handle;
	esp_err_t ok_op = nvs_open("lbm", NVS_READWRITE, &my_handle);
	esp_err_t ok_set = nvs_set_u32(my_handle, buf, v->as_u32);
	esp_err_t ok_com = nvs_commit(my_handle);
	nvs_close(my_handle);

	return ok_op == ESP_OK && ok_set == ESP_OK && ok_com == ESP_OK;
}

static bool read_eeprom_var(eeprom_var *v, int address) {
	if (address < 0 || address > 127) {
		return false;
	}

	char buf[10];
	sprintf(buf, "v%d", address);

	nvs_handle_t my_handle;
	esp_err_t ok_op = nvs_open("lbm", NVS_READONLY, &my_handle);
	esp_err_t ok_set = nvs_get_u32(my_handle, buf, &v->as_u32);
	nvs_close(my_handle);

	return ok_op == ESP_OK && ok_set == ESP_OK;
}

static lbm_value ext_eeprom_store_f(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	v.as_float = lbm_dec_as_float(args[1]);
	return store_eeprom_var(&v, addr) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_read_f(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	bool res = read_eeprom_var(&v, addr);
	return res ? lbm_enc_float(v.as_float) : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_store_i(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	v.as_i32 = lbm_dec_as_i32(args[1]);
	return store_eeprom_var(&v, addr) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_eeprom_read_i(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	eeprom_var v;
	bool res = read_eeprom_var(&v, addr);
	return res ? lbm_enc_i32(v.as_i32) : ENC_SYM_NIL;
}

static lbm_uint sym_hw_express;

static lbm_value ext_sysinfo(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	if (argn != 1) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name = lbm_dec_sym(args[0]);

	if (compare_symbol(name, &syms_vesc.hw_name)) {
		lbm_value lbm_res;
		if (lbm_create_array(&lbm_res, LBM_TYPE_CHAR, strlen(HW_NAME) + 1)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(lbm_res);
			strcpy((char*)arr->data, HW_NAME);
			res = lbm_res;
		} else {
			res = ENC_SYM_MERROR;
		}
	} else if (compare_symbol(name, &syms_vesc.fw_ver)) {
		res = ENC_SYM_NIL;
		res = lbm_cons(lbm_enc_i(FW_TEST_VERSION_NUMBER), res);
		res = lbm_cons(lbm_enc_i(FW_VERSION_MINOR), res);
		res = lbm_cons(lbm_enc_i(FW_VERSION_MAJOR), res);
	} else if (compare_symbol(name, &syms_vesc.hw_type)) {
		res = lbm_enc_sym(sym_hw_express);
	}

	return res;
}

static lbm_value ext_can_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	if (!lbm_is_number(args[0])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_EERROR;
	}

	int id = lbm_dec_as_i32(args[0]);
	if (id < 0 || id > 255) {
		return ENC_SYM_EERROR;
	}

	char *str = lbm_dec_str(args[1]);
	if (!str) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_EERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);

	if (array->size > 500) {
		return ENC_SYM_EERROR;
	}

	uint8_t *send_buf = mempools_get_packet_buffer();
	send_buf[0] = COMM_LISP_REPL_CMD;
	memcpy(send_buf + 1, array->data, array->size);
	comm_can_send_buffer(id, send_buf, array->size + 1, 2);
	mempools_free_packet_buffer(send_buf);

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_get_current(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg *stat0 = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
	if (stat0) {
		return lbm_enc_float(stat0->current);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_current_dir(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg *stat0 = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
	if (stat0) {
		return lbm_enc_float(stat0->current * SIGN(stat0->duty));
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_current_in(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_4 *stat4 = comm_can_get_status_msg_4_id(lbm_dec_as_i32(args[0]));
	if (stat4) {
		return lbm_enc_float((float)stat4->current_in);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_duty(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg *stat0 = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
	if (stat0) {
		return lbm_enc_float(stat0->duty);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_rpm(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg *stat0 = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
	if (stat0) {
		return lbm_enc_float(stat0->rpm);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_temp_fet(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_4 *stat4 = comm_can_get_status_msg_4_id(lbm_dec_as_i32(args[0]));
	if (stat4) {
		return lbm_enc_float((float)stat4->temp_fet);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_temp_motor(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_4 *stat4 = comm_can_get_status_msg_4_id(lbm_dec_as_i32(args[0]));
	if (stat4) {
		return lbm_enc_float((float)stat4->temp_motor);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_speed(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg *stat0 = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
	if (stat0) {
		return lbm_enc_float(stat0->rpm);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_dist(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_5 *stat5 = comm_can_get_status_msg_5_id(lbm_dec_as_i32(args[0]));
	if (stat5) {
		const float tacho_scale = 1.0;
		return lbm_enc_float((float)stat5->tacho_value * tacho_scale);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_ppm(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_6 *stat6 = comm_can_get_status_msg_6_id(lbm_dec_as_i32(args[0]));
	if (stat6) {
		return lbm_enc_float((float)stat6->ppm);
	} else {
		return lbm_enc_float(0.0);
	}
}

static lbm_value ext_can_get_adc(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && argn != 2) {
		return ENC_SYM_EERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	lbm_int channel = 0;
	if (argn == 2) {
		channel = lbm_dec_as_i32(args[1]);
	}

	can_status_msg_6 *stat6 = comm_can_get_status_msg_6_id(lbm_dec_as_i32(args[0]));

	if (stat6) {
		if (channel == 0) {
			return lbm_enc_float(stat6->adc_1);
		} else if (channel == 1) {
			return lbm_enc_float(stat6->adc_2);
		} else if (channel == 2) {
			return lbm_enc_float(stat6->adc_3);
		} else {
			return ENC_SYM_EERROR;
		}
	} else {
		return lbm_enc_float(-1.0);
	}
}

static lbm_value ext_can_get_vin(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	can_status_msg_5 *stat5 = comm_can_get_status_msg_5_id(lbm_dec_as_i32(args[0]));
	if (stat5) {
		return lbm_enc_float(stat5->v_in);
	} else {
		return lbm_enc_float(0.0);
	}
}

static int cmp_int (const void * a, const void * b) {
	return ( *(int*)a - *(int*)b );
}

static lbm_value ext_can_list_devs(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	int dev_num = 0;
	can_status_msg *msg = comm_can_get_status_msg_index(dev_num);

	while (msg && msg->id >= 0) {
		dev_num++;
		msg = comm_can_get_status_msg_index(dev_num);
	}

	int devs[dev_num];

	for (int i = 0;i < dev_num;i++) {
		msg = comm_can_get_status_msg_index(i);
		if (msg) {
			devs[i] = msg->id;
		} else {
			devs[i] = -1;
		}
	}

	qsort(devs, dev_num, sizeof(int), cmp_int);
	lbm_value dev_list = ENC_SYM_NIL;

	for (int i = (dev_num - 1);i >= 0;i--) {
		if (devs[i] >= 0) {
			dev_list = lbm_cons(lbm_enc_i(devs[i]), dev_list);
		} else {
			break;
		}
	}

	return dev_list;
}

static lbm_value ext_can_scan(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	lbm_value dev_list = ENC_SYM_NIL;

	for (int i = 253;i >= 0;i--) {
		if (comm_can_ping(i, 0)) {
			dev_list = lbm_cons(lbm_enc_i(i), dev_list);
		}
	}

	return dev_list;
}

static lbm_value ext_can_send(lbm_value *args, lbm_uint argn, bool is_eid) {
	if (argn != 2 || !lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}

	lbm_value curr = args[1];
	uint8_t to_send[8];
	int ind = 0;

	if (lbm_type_of(curr) == LBM_TYPE_ARRAY) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(curr);
		if (array->elt_type != LBM_TYPE_BYTE) {
			return ENC_SYM_EERROR;
		}

		ind = array->size;
		if (ind > 8) {
			ind = 0;
		}

		memcpy(to_send, array->data, ind);
	} else {
		while (lbm_is_cons(curr)) {
			lbm_value  arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[ind++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (ind == 8) {
				break;
			}

			curr = lbm_cdr(curr);
		}
	}

	if (is_eid) {
		comm_can_transmit_eid(lbm_dec_as_u32(args[0]), to_send, ind);
	} else {
		comm_can_transmit_sid(lbm_dec_as_u32(args[0]), to_send, ind);
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_send_sid(lbm_value *args, lbm_uint argn) {
	return ext_can_send(args, argn, false);
}

static lbm_value ext_can_send_eid(lbm_value *args, lbm_uint argn) {
	return ext_can_send(args, argn, true);
}

static lbm_value ext_can_current(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn == 2) {
		comm_can_set_current(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	} else if (argn == 3) {
		comm_can_set_current_off_delay(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]), lbm_dec_as_float(args[2]));
	} else {
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_current_rel(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn == 2) {
		comm_can_set_current_rel(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	} else if (argn == 3) {
		comm_can_set_current_rel_off_delay(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]), lbm_dec_as_float(args[2]));
	} else {
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_duty(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	comm_can_set_duty(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_can_brake(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	comm_can_set_current_brake(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_can_brake_rel(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	comm_can_set_current_brake_rel(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_can_rpm(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	comm_can_set_rpm(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_can_pos(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	comm_can_set_pos(lbm_dec_as_i32(args[0]), lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

// Bit operations

/*
 * args[0]: Initial value
 * args[1]: Offset in initial value to modify
 * args[2]: Value to modify with
 * args[3]: Size in bits of value to modify with
 */
static lbm_value ext_bits_enc_int(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4)
	uint32_t initial = lbm_dec_as_u32(args[0]);
	uint32_t offset = lbm_dec_as_u32(args[1]);
	uint32_t number = lbm_dec_as_u32(args[2]);
	uint32_t bits = lbm_dec_as_u32(args[3]);
	initial &= ~((0xFFFFFFFF >> (32 - bits)) << offset);
	initial |= (number << (32 - bits)) >> (32 - bits - offset);

	if (initial > ((1 << 27) - 1)) {
		return lbm_enc_i32(initial);
	} else {
		return lbm_enc_i(initial);
	}
}

/*
 * args[0]: Value
 * args[1]: Offset in initial value to get
 * args[2]: Size in bits of value to get
 */
static lbm_value ext_bits_dec_int(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3)
	uint32_t val = lbm_dec_as_u32(args[0]);
	uint32_t offset = lbm_dec_as_u32(args[1]);
	uint32_t bits = lbm_dec_as_u32(args[2]);
	val >>= offset;
	val &= 0xFFFFFFFF >> (32 - bits);

	if (val > ((1 << 27) - 1)) {
		return lbm_enc_i32(val);
	} else {
		return lbm_enc_i(val);
	}
}

// Events that will be sent to lisp if a handler is registered

static volatile bool event_can_sid_en = false;
static volatile bool event_can_eid_en = false;
static volatile bool event_data_rx_en = false;
static volatile bool event_esp_now_rx_en = false;
static lbm_uint sym_event_can_sid;
static lbm_uint sym_event_can_eid;
static lbm_uint sym_event_data_rx;
static lbm_uint sym_event_esp_now_rx;

static lbm_value ext_enable_event(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && argn != 2) {
		return ENC_SYM_EERROR;
	}

	if (argn == 2 && !lbm_is_number(args[1])) {
		return ENC_SYM_EERROR;
	}

	bool en = true;
	if (argn == 2 && !lbm_dec_as_i32(args[1])) {
		en = false;
	}

	lbm_uint name = lbm_dec_sym(args[0]);

	if (name == sym_event_can_sid) {
		event_can_sid_en = en;
	} else if (name == sym_event_can_eid) {
		event_can_eid_en = en;
	} else if (name == sym_event_data_rx) {
		event_data_rx_en = en;
	} else if (name == sym_event_esp_now_rx) {
		event_esp_now_rx_en = en;
	} else {
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value make_list(int num, ...) {
	va_list arguments;
	va_start (arguments, num);
	lbm_value res = ENC_SYM_NIL;
	for (int i = 0; i < num; i++) {
		res = lbm_cons(va_arg(arguments, lbm_value), res);
	}
	va_end (arguments);
	return lbm_list_destructive_reverse(res);
}

static lbm_uint sym_res;
static lbm_uint sym_loop;
static lbm_uint sym_break;
static lbm_uint sym_brk;
static lbm_uint sym_rst;

static lbm_value ext_me_defun(lbm_value *argsi, lbm_uint argn) {
	if (argn != 3) {
		return ENC_SYM_EERROR;
	}

	lbm_value name = argsi[0];
	lbm_value args = argsi[1];
	lbm_value body = argsi[2];

	// (define name (lambda args body))

	return make_list(3,
			lbm_enc_sym(SYM_DEFINE),
			name,
			make_list(3,
					lbm_enc_sym(SYM_LAMBDA),
					args,
					body));
}

static lbm_value ext_me_loopfor(lbm_value *args, lbm_uint argn) {
	if (argn != 5) {
		return ENC_SYM_EERROR;
	}

	lbm_value it = args[0];
	lbm_value start = args[1];
	lbm_value cond = args[2];
	lbm_value update = args[3];
	lbm_value body = args[4];

	// (let ((loop (lambda (it res break) (if cond (loop update body break) res)))) (call-cc (lambda (brk) (loop start nil brk))))

	return make_list(3,
			lbm_enc_sym(SYM_LET),
			make_list(1,
					make_list(2,
							lbm_enc_sym(sym_loop),
							make_list(3,
									lbm_enc_sym(SYM_LAMBDA),
									make_list(3, it, lbm_enc_sym(sym_res), lbm_enc_sym(sym_break)),
									make_list(4,
											lbm_enc_sym(SYM_IF),
											cond,
											make_list(4, lbm_enc_sym(sym_loop), update, body, lbm_enc_sym(sym_break)),
											lbm_enc_sym(sym_res))))),
											make_list(2,
													lbm_enc_sym(SYM_CALLCC),
													make_list(3,
															lbm_enc_sym(SYM_LAMBDA),
															make_list(1, lbm_enc_sym(sym_brk)),
															make_list(4, lbm_enc_sym(sym_loop), start, ENC_SYM_NIL, lbm_enc_sym(sym_brk)))));
}

static lbm_value ext_me_loopwhile(lbm_value *args, lbm_uint argn) {
	if (argn != 2) {
		return ENC_SYM_EERROR;
	}

	lbm_value cond = args[0];
	lbm_value body = args[1];

	// (let ((loop (lambda (res break) (if cond (loop body break) res)))) (call-cc (lambda (brk) (loop nil brk))))

	return make_list(3,
			lbm_enc_sym(SYM_LET),
			make_list(1,
					make_list(2,
							lbm_enc_sym(sym_loop),
							make_list(3,
									lbm_enc_sym(SYM_LAMBDA),
									make_list(2, lbm_enc_sym(sym_res), lbm_enc_sym(sym_break)),
									make_list(4,
											lbm_enc_sym(SYM_IF),
											cond,
											make_list(3, lbm_enc_sym(sym_loop), body, lbm_enc_sym(sym_break)),
											lbm_enc_sym(sym_res))))),
											make_list(2,
													lbm_enc_sym(SYM_CALLCC),
													make_list(3,
															lbm_enc_sym(SYM_LAMBDA),
															make_list(1, lbm_enc_sym(sym_brk)),
															make_list(3, lbm_enc_sym(sym_loop), ENC_SYM_NIL, lbm_enc_sym(sym_brk)))));
}

static lbm_value ext_me_looprange(lbm_value *args, lbm_uint argn) {
	if (argn != 4) {
		return ENC_SYM_EERROR;
	}

	lbm_value it = args[0];
	lbm_value start = args[1];
	lbm_value end = args[2];
	lbm_value body = args[3];

	// (let ((loop (lambda (it res break) (if (< it end) (loop (+ it 1) body break) res)))) (call-cc (lambda (brk) (loop start nil brk))))

	return make_list(3,
			lbm_enc_sym(SYM_LET),
			make_list(1,
					make_list(2,
							lbm_enc_sym(sym_loop),
							make_list(3,
									lbm_enc_sym(SYM_LAMBDA),
									make_list(3, it, lbm_enc_sym(sym_res), lbm_enc_sym(sym_break)),
									make_list(4,
											lbm_enc_sym(SYM_IF),
											make_list(3, lbm_enc_sym(SYM_LT), it, end),
											make_list(4, lbm_enc_sym(sym_loop), make_list(3, lbm_enc_sym(SYM_ADD), it, lbm_enc_i(1)), body, lbm_enc_sym(sym_break)),
											lbm_enc_sym(sym_res))))),
											make_list(2,
													lbm_enc_sym(SYM_CALLCC),
													make_list(3,
															lbm_enc_sym(SYM_LAMBDA),
															make_list(1, lbm_enc_sym(sym_brk)),
															make_list(4, lbm_enc_sym(sym_loop), start, ENC_SYM_NIL, lbm_enc_sym(sym_brk)))));
}

static lbm_value ext_me_loopforeach(lbm_value *args, lbm_uint argn) {
	if (argn != 3) {
		return ENC_SYM_EERROR;
	}

	lbm_value it = args[0];
	lbm_value lst = args[1];
	lbm_value body = args[2];

	// (let ((loop (lambda (it rst res break) (if (eq it nil) res (loop (car rst) (cdr rst) body break))))) (call-cc (lambda (brk) (loop (car lst) (cdr lst) nil brk))))

	return make_list(3,
			lbm_enc_sym(SYM_LET),
			make_list(1,
					make_list(2,
							lbm_enc_sym(sym_loop),
							make_list(3,
									lbm_enc_sym(SYM_LAMBDA),
									make_list(4, it, lbm_enc_sym(sym_rst), lbm_enc_sym(sym_res), lbm_enc_sym(sym_break)),
									make_list(4,
											lbm_enc_sym(SYM_IF),
											make_list(3, lbm_enc_sym(SYM_EQ), it, ENC_SYM_NIL),
											lbm_enc_sym(sym_res),
											make_list(5,
													lbm_enc_sym(sym_loop),
													make_list(2, lbm_enc_sym(SYM_CAR), lbm_enc_sym(sym_rst)),
													make_list(2, lbm_enc_sym(SYM_CDR), lbm_enc_sym(sym_rst)),
													body,
													lbm_enc_sym(sym_break))
											)))),
											make_list(2,
													lbm_enc_sym(SYM_CALLCC),
													make_list(3,
															lbm_enc_sym(SYM_LAMBDA),
															make_list(1, lbm_enc_sym(sym_brk)),
															make_list(5,
																	lbm_enc_sym(sym_loop),
																	make_list(2, lbm_enc_sym(SYM_CAR), lst),
																	make_list(2, lbm_enc_sym(SYM_CDR), lst),
																	ENC_SYM_NIL,
																	lbm_enc_sym(sym_brk)))));
}

static lbm_value ext_lbm_set_quota(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	uint32_t q = lbm_dec_as_u32(args[0]);

	if (q < 1) {
		return ENC_SYM_EERROR;
	}

	lbm_set_eval_step_quota(q);
	return ENC_SYM_TRUE;
}

static lbm_value ext_plot_init(lbm_value *args, lbm_uint argn) {
	if (argn != 2) {
		return ENC_SYM_EERROR;
	}

	char *namex = lbm_dec_str(args[0]);
	if (!namex) {
		return ENC_SYM_EERROR;
	}

	char *namey = lbm_dec_str(args[1]);
	if (!namey) {
		return ENC_SYM_EERROR;
	}

	commands_init_plot(namex, namey);

	return ENC_SYM_TRUE;
}

static lbm_value ext_plot_add_graph(lbm_value *args, lbm_uint argn) {
	if (argn != 1) {
		return ENC_SYM_EERROR;
	}

	char *name = lbm_dec_str(args[0]);
	if (!name) {
		return ENC_SYM_EERROR;
	}

	commands_plot_add_graph(name);

	return ENC_SYM_TRUE;
}

static lbm_value ext_plot_set_graph(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	commands_plot_set_graph(lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_plot_send_points(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);
	commands_send_plot_points(
			lbm_dec_as_float(args[0]),
			lbm_dec_as_float(args[1]));
	return ENC_SYM_TRUE;
}

// IO-boards

static lbm_value ext_ioboard_get_adc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int id = lbm_dec_as_i32(args[0]);
	int channel = lbm_dec_as_i32(args[1]);

	if (channel < 1 || channel > 8) {
		lbm_set_error_reason("Channel must be 1 - 8");
		return ENC_SYM_EERROR;
	}

	io_board_adc_values *val = 0;
	if (channel >= 5) {
		val = comm_can_get_io_board_adc_5_8_id(id);
		channel -= 4;
	} else {
		val = comm_can_get_io_board_adc_1_4_id(id);
	}

	if (val) {
		return lbm_enc_float(val->adc_voltages[channel - 1]);
	} else {
		return lbm_enc_float(-1.0);
	}
}

static lbm_value ext_ioboard_get_digital(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int id = lbm_dec_as_i32(args[0]);
	int channel = lbm_dec_as_i32(args[1]);

	if (channel < 1 || channel > 64) {
		lbm_set_error_reason("Channel must be 1 - 64");
		return ENC_SYM_EERROR;
	}

	io_board_digial_inputs *val = comm_can_get_io_board_digital_in_id(id);

	if (val) {
		return lbm_enc_i(val->inputs >> (channel - 1));
	} else {
		return lbm_enc_i(-1);
	}
}

static lbm_value ext_ioboard_set_digital(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);
	int id = lbm_dec_as_i32(args[0]);
	int channel = lbm_dec_as_i32(args[1]);
	bool on = lbm_dec_as_i32(args[2]);
	comm_can_io_board_set_output_digital(id, channel, on);
	return ENC_SYM_TRUE;
}

static lbm_value ext_ioboard_set_pwm(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);
	int id = lbm_dec_as_i32(args[0]);
	int channel = lbm_dec_as_i32(args[1]);
	float duty = lbm_dec_as_float(args[2]);
	comm_can_io_board_set_output_pwm(id, channel, duty);
	return ENC_SYM_TRUE;
}

// ESP NOW

static bool esp_now_initialized = false;
static volatile lbm_cid esp_now_send_cid;
static char *esp_init_msg = "ESP-NOW not initialized";

typedef struct {
	uint8_t *data;
	int len;
	uint8_t src[6];
	uint8_t des[6];
} esp_now_send_data;

#define ESP_NOW_RX_BUFFER_ELEMENTS		10
static rb_t esp_now_rx_rb;
static esp_now_send_data esp_now_rx_data[ESP_NOW_RX_BUFFER_ELEMENTS];
static SemaphoreHandle_t esp_now_rx_sem;

static void esp_rx_fun(void *arg) {
	(void)arg;

	for (;;) {
		xSemaphoreTake(esp_now_rx_sem, 10 / portTICK_PERIOD_MS);

		esp_now_send_data data;
		if (!rb_pop(&esp_now_rx_rb, &data)) {
			continue;
		}

		lbm_flat_value_t v;
		if (lbm_start_flatten(&v, 150 + data.len)) {
			f_cons(&v);
			f_sym(&v, sym_event_esp_now_rx);

			f_cons(&v);
			for (int i = 0; i < 6; i++) {
				f_cons(&v);
				f_i(&v, data.src[i]);
			}
			f_sym(&v, SYM_NIL);

			f_cons(&v);
			for (int i = 0; i < 6; i++) {
				f_cons(&v);
				f_i(&v, data.des[i]);
			}
			f_sym(&v, SYM_NIL);

			f_cons(&v);
			f_lbm_array(&v, data.len, LBM_TYPE_BYTE, data.data);

			f_sym(&v, SYM_NIL);

			lbm_finish_flatten(&v);

			if (!lbm_event(&v)) {
				lbm_free(v.buf);
			}
		}

		free(data.data);
	}
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
	lbm_unblock_ctx_unboxed(esp_now_send_cid, status == ESP_NOW_SEND_SUCCESS ? ENC_SYM_TRUE : ENC_SYM_NIL);
}

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
	if (event_esp_now_rx_en) {
		esp_now_send_data sdata;

		sdata.data = malloc(data_len);
		if (!sdata.data) {
			return;
		}

		sdata.len = data_len;
		memcpy(sdata.data, data, data_len);
		memcpy(sdata.src, esp_now_info->src_addr, 6);
		memcpy(sdata.des, esp_now_info->des_addr, 6);

		if (rb_insert(&esp_now_rx_rb, &sdata)) {
			xSemaphoreGive(esp_now_rx_sem);
		} else {
			free(sdata.data);
		}
	}
}

static lbm_value ext_esp_now_start(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	main_wait_until_init_done();

	if (backup.config.wifi_mode == WIFI_MODE_DISABLED && !esp_now_initialized) {
		esp_netif_init();
		esp_event_loop_create_default();
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		esp_wifi_init(&cfg);
		esp_wifi_set_storage(WIFI_STORAGE_RAM);
		esp_wifi_set_mode(WIFI_MODE_AP);

		// Disable power save mode. Does not work with bluetooth.
		if (backup.config.ble_mode == BLE_MODE_DISABLED) {
			esp_wifi_set_ps(WIFI_PS_NONE);
		}

		esp_wifi_start();
	}

	if (!esp_now_initialized) {
		if (esp_now_init() != ESP_OK) {
			return ENC_SYM_EERROR;
		}

		esp_now_rx_sem = xSemaphoreCreateBinary();
		rb_init(&esp_now_rx_rb, esp_now_rx_data, sizeof(esp_now_send_data), ESP_NOW_RX_BUFFER_ELEMENTS);
		xTaskCreate(esp_rx_fun, "esp_rx", 2048, NULL, 3, NULL);

		esp_now_register_send_cb(espnow_send_cb);
		esp_now_register_recv_cb(espnow_recv_cb);
		esp_now_initialized = true;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_esp_now_add_peer(lbm_value *args, lbm_uint argn) {
	if (!esp_now_initialized) {
		lbm_set_error_reason(esp_init_msg);
		return ENC_SYM_EERROR;
	}

	if (argn != 1 || !lbm_is_list(args[0])) {
		return ENC_SYM_EERROR;
	}

	uint8_t addr[ESP_NOW_ETH_ALEN] = {255, 255, 255, 255, 255, 255};
	int ind = 0;

	lbm_value curr = args[0];
	while (lbm_is_cons(curr)) {
		lbm_value  arg = lbm_car(curr);

		if (lbm_is_number(arg)) {
			addr[ind++] = lbm_dec_as_u32(arg);
		} else {
			return ENC_SYM_TERROR;
		}

		if (ind == ESP_NOW_ETH_ALEN) {
			break;
		}

		curr = lbm_cdr(curr);
	}

	esp_now_peer_info_t peer;
	memset(&peer, 0, sizeof(peer));
	peer.channel = 0; // Must be the same as the wifi-channel when using wifi. 0 means current channel.
	peer.ifidx = ESP_IF_WIFI_AP;
	peer.encrypt = false;
	memcpy(peer.peer_addr, addr, ESP_NOW_ETH_ALEN);

	esp_err_t res = esp_now_add_peer(&peer);

	if (res == ESP_OK || res == ESP_ERR_ESPNOW_EXIST) {
		return ENC_SYM_TRUE;
	} else {
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_get_mac_addr(lbm_value *args, lbm_uint argn) {
	(void) args; (void) argn;

	uint8_t mac[ESP_NOW_ETH_ALEN];
	esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

	lbm_value addr = ENC_SYM_NIL;
	for (int i = (ESP_NOW_ETH_ALEN - 1); i >= 0; i--) {
		addr = lbm_cons(lbm_enc_i(mac[i]), addr);
	}

	return addr;
}

static char *str_wifi_not_init_msg = "WiFi not initialized.";

static lbm_value ext_wifi_set_chan(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t ch = lbm_dec_as_i32(args[0]);

	if (ch > 14) {
		return ENC_SYM_TERROR;
	}

	uint8_t prim;
	wifi_second_chan_t second;
	esp_err_t res = esp_wifi_get_channel(&prim, &second);

	if (res == ESP_ERR_WIFI_NOT_INIT) {
		lbm_set_error_reason(str_wifi_not_init_msg);
		return ENC_SYM_EERROR;
	}

	esp_wifi_set_channel(ch, second);

	return ENC_SYM_TRUE;
}

static lbm_value ext_wifi_get_chan(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	uint8_t prim;
	wifi_second_chan_t second;
	esp_err_t res = esp_wifi_get_channel(&prim, &second);

	if (res == ESP_ERR_WIFI_NOT_INIT) {
		lbm_set_error_reason(str_wifi_not_init_msg);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(prim);
}

static lbm_value ext_wifi_set_bw(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint8_t bw = lbm_dec_as_i32(args[0]);

	if (bw != 20 && bw != 40) {
		return ENC_SYM_TERROR;
	}

	wifi_bandwidth_t bwt = WIFI_BW_HT20;
	if (bw == 40) {
		bwt = WIFI_BW_HT40;
	}

	esp_err_t res = esp_wifi_set_bandwidth(WIFI_IF_AP, bwt);

	if (res == ESP_ERR_WIFI_NOT_INIT) {
		lbm_set_error_reason(str_wifi_not_init_msg);
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_wifi_get_bw(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	wifi_bandwidth_t bwt = WIFI_BW_HT20;
	esp_err_t res = esp_wifi_get_bandwidth(WIFI_IF_AP, &bwt);

	if (res == ESP_ERR_WIFI_NOT_INIT) {
		lbm_set_error_reason(str_wifi_not_init_msg);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(bwt == WIFI_BW_HT20 ? 20 : 40);
}

static lbm_value ext_esp_now_send(lbm_value *args, lbm_uint argn) {
	if (!esp_now_initialized) {
		lbm_set_error_reason(esp_init_msg);
		return ENC_SYM_EERROR;
	}

	if (argn != 2) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	uint8_t peer[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	int ind = 0;

	lbm_value curr = args[0];
	while (lbm_is_cons(curr)) {
		lbm_value  arg = lbm_car(curr);

		if (lbm_is_number(arg)) {
			peer[ind++] = lbm_dec_as_u32(arg);
		} else {
			return ENC_SYM_TERROR;
		}

		if (ind == ESP_NOW_ETH_ALEN) {
			break;
		}

		curr = lbm_cdr(curr);
	}

	char *str = lbm_dec_str(args[1]);
	if (str) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		esp_now_send_cid = lbm_get_current_cid();
		lbm_block_ctx_from_extension();
		esp_err_t send_res = esp_now_send(peer, (uint8_t*)str, (size_t)array->size);

		if (send_res != ESP_OK) {
			lbm_undo_block_ctx_from_extension();
			return ENC_SYM_NIL;
		}
	} else {
		lbm_set_error_reason("Argument must be an array");
		return ENC_SYM_TERROR;
	}

	return ENC_SYM_TRUE;
}

static bool i2c_started = false;
static SemaphoreHandle_t i2c_mutex;
static bool i2c_mutex_init_done = false;

static lbm_value ext_i2c_start(lbm_value *args, lbm_uint argn) {
	if (argn > 3) {
		return ENC_SYM_EERROR;
	}

	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = 7,
			.scl_io_num = 6,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 200000,
	};

	if (argn >= 1) {
		if (!lbm_is_symbol(args[0])) {
			return ENC_SYM_EERROR;
		}

		if (compare_symbol(lbm_dec_sym(args[0]), &syms_vesc.rate_100k)) {
			conf.master.clk_speed = 100000;
		} else if (compare_symbol(lbm_dec_sym(args[0]), &syms_vesc.rate_200k)) {
			conf.master.clk_speed = 200000;
		} else if (compare_symbol(lbm_dec_sym(args[0]), &syms_vesc.rate_400k)) {
			conf.master.clk_speed = 400000;
		} else if (compare_symbol(lbm_dec_sym(args[0]), &syms_vesc.rate_700k)) {
			conf.master.clk_speed = 700000;
		} else {
			return ENC_SYM_EERROR;
		}
	}

	if (argn >= 2) {
		if (!lbm_is_number(args[1])) {
			return ENC_SYM_EERROR;
		}

		conf.sda_io_num = lbm_dec_as_i32(args[1]);
	}

	if (argn >= 3) {
		if (!lbm_is_number(args[2])) {
			return ENC_SYM_EERROR;
		}

		conf.scl_io_num = lbm_dec_as_i32(args[2]);
	}

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);
	i2c_started = true;

	return ENC_SYM_TRUE;
}

static esp_err_t i2c_tx_rx(uint8_t addr,
		const uint8_t* write_buffer, size_t write_size,
		uint8_t* read_buffer, size_t read_size) {

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	esp_err_t res;
	if (read_buffer != NULL) {
		res = i2c_master_write_read_device(0, addr, write_buffer, write_size, read_buffer, read_size, 2000);
	} else {
		res = i2c_master_write_to_device(0, addr, write_buffer, write_size, 2000);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static lbm_value ext_i2c_tx_rx(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		return ENC_SYM_EERROR;
	}

	if (!i2c_started) {
		return lbm_enc_i(0);
	}

	uint16_t addr = 0;
	size_t txlen = 0;
	size_t rxlen = 0;
	uint8_t *txbuf = 0;
	uint8_t *rxbuf = 0;

	const unsigned int max_len = 20;
	uint8_t to_send[max_len];

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}
	addr = lbm_dec_as_u32(args[0]);

	if (lbm_type_of(args[1]) == LBM_TYPE_ARRAY) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		if (array->elt_type != LBM_TYPE_BYTE) {
			return ENC_SYM_EERROR;
		}

		txbuf = (uint8_t*)array->data;
		txlen = array->size;
	} else {
		lbm_value curr = args[1];
		while (lbm_is_cons(curr)) {
			lbm_value  arg = lbm_car(curr);

			if (lbm_is_number(arg)) {
				to_send[txlen++] = lbm_dec_as_u32(arg);
			} else {
				return ENC_SYM_EERROR;
			}

			if (txlen == max_len) {
				break;
			}

			curr = lbm_cdr(curr);
		}

		if (txlen > 0) {
			txbuf = to_send;
		}
	}

	if (argn >= 3 && lbm_type_of(args[2]) == LBM_TYPE_ARRAY) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);
		if (array->elt_type != LBM_TYPE_BYTE) {
			return ENC_SYM_EERROR;
		}

		rxbuf = (uint8_t*)array->data;
		rxlen = array->size;
	}

	return lbm_enc_i(i2c_tx_rx(addr, txbuf, txlen, rxbuf, rxlen));
}

static bool gpio_is_valid(int pin) {
	switch (pin) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 18:
	case 19:
	case 20:
	case 21:
		return true;

	default:
		return false;
	}
}

static char *pin_invalid_msg = "Invalid pin";

static lbm_value ext_gpio_configure(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	if (!lbm_is_number(args[0]) || !lbm_is_symbol(args[1])) {
		return ENC_SYM_EERROR;
	}

	int pin = lbm_dec_as_i32(args[0]);
	lbm_uint name = lbm_dec_sym(args[1]);

	if (!gpio_is_valid(pin)) {
		lbm_set_error_reason(pin_invalid_msg);
		return ENC_SYM_EERROR;
	}

	gpio_config_t gpconf = {0};

	gpconf.pin_bit_mask = BIT(pin);
	gpconf.intr_type =  GPIO_FLOATING;

	if (compare_symbol(name, &syms_vesc.pin_mode_out)) {
		gpconf.mode = GPIO_MODE_INPUT_OUTPUT;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else if (compare_symbol(GPIO_MODE_INPUT_OUTPUT_OD, &syms_vesc.pin_mode_od)) {
		gpconf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_od_pu)) {
		gpconf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_ENABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_od_pd)) {
		gpconf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
		gpconf.pull_down_en = GPIO_PULLDOWN_ENABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_in)) {
		gpconf.mode = GPIO_MODE_INPUT;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_in_pu)) {
		gpconf.mode = GPIO_MODE_INPUT;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_ENABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_in_pd)) {
		gpconf.mode = GPIO_MODE_INPUT;
		gpconf.pull_down_en = GPIO_PULLDOWN_ENABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else if (compare_symbol(name, &syms_vesc.pin_mode_analog)) {
		gpconf.mode = GPIO_MODE_DISABLE;
		gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	} else {
		lbm_set_error_reason("Invalid pin mode");
		return ENC_SYM_EERROR;
	}

	gpio_reset_pin(pin);
	gpio_config(&gpconf);

	return ENC_SYM_TRUE;
}

static lbm_value ext_gpio_write(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	int state = lbm_dec_as_i32(args[1]);

	if (!gpio_is_valid(pin)) {
		lbm_set_error_reason(pin_invalid_msg);
		return ENC_SYM_EERROR;
	}

	gpio_set_level(pin, state);

	return ENC_SYM_TRUE;
}

static lbm_value ext_gpio_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int pin = lbm_dec_as_i32(args[0]);
	if (!gpio_is_valid(pin)) {
		lbm_set_error_reason(pin_invalid_msg);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(gpio_get_level(pin));
}

static lbm_value ext_main_init_done(lbm_value *args, lbm_uint argn) {
	(void)args;(void)argn;
	return main_init_done() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_crc16(lbm_value *args, lbm_uint argn) {
	if ((argn != 1 && argn != 2) || !lbm_is_array(args[0])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	if (array->elt_type != LBM_TYPE_BYTE) {
		return ENC_SYM_TERROR;
	}

	unsigned int len = array->size;
	if (argn == 2) {
		if (!lbm_is_number(args[1])) {
			return ENC_SYM_TERROR;
		}

		len = lbm_dec_as_u32(args[1]);
		if (len > array->size) {
			len = array->size;
		}
	}

	return lbm_enc_i(crc16((uint8_t*)array->data, len));
}

// WS2812-driver using RMT

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

typedef struct {
	rmt_encoder_t base;
	rmt_encoder_t *bytes_encoder;
	rmt_encoder_t *copy_encoder;
	int state;
	rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static uint8_t *led_pixels = NULL;
static int led_num = -1;

static rmt_transmit_config_t tx_config = {
		.loop_count = 0, // no transfer loop
};

static size_t rmt_encode_led_strip(
		rmt_encoder_t *encoder, rmt_channel_handle_t channel,
		const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
	rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
	rmt_encode_state_t session_state = 0;
	rmt_encode_state_t state = 0;
	size_t encoded_symbols = 0;
	switch (led_encoder->state) {
	case 0: // send RGB data
		encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = 1; // switch to next state when current encoding session finished
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out; // yield if there's no free space for encoding artifacts
		}
		// fall-through
		//no break
	case 1: // send reset code
		encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
				sizeof(led_encoder->reset_code), &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = 0; // back to the initial encoding session
			state |= RMT_ENCODING_COMPLETE;
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= RMT_ENCODING_MEM_FULL;
			goto out; // yield if there's no free space for encoding artifacts
		}
	}
	out:
	*ret_state = state;
	return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_del_encoder(led_encoder->bytes_encoder);
	rmt_del_encoder(led_encoder->copy_encoder);
	free(led_encoder);
	return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
	rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
	rmt_encoder_reset(led_encoder->bytes_encoder);
	rmt_encoder_reset(led_encoder->copy_encoder);
	led_encoder->state = 0;
	return ESP_OK;
}

esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder) {
	rmt_led_strip_encoder_t *led_encoder = NULL;
	led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
	led_encoder->base.encode = rmt_encode_led_strip;
	led_encoder->base.del = rmt_del_led_strip_encoder;
	led_encoder->base.reset = rmt_led_strip_encoder_reset;

	// different led strip might have its own timing requirements, following parameter is for WS2812
	rmt_bytes_encoder_config_t bytes_encoder_config = {
			.bit0 = {
					.level0 = 1,
					.duration0 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0H=0.3us
					.level1 = 0,
					.duration1 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T0L=0.9us
			},
			.bit1 = {
					.level0 = 1,
					.duration0 = 0.9 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1H=0.9us
					.level1 = 0,
					.duration1 = 0.3 * RMT_LED_STRIP_RESOLUTION_HZ / 1000000, // T1L=0.3us
			},
			.flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
	};

	rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
	rmt_copy_encoder_config_t copy_encoder_config = {};
	rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);

	uint32_t reset_ticks = RMT_LED_STRIP_RESOLUTION_HZ / 1000000 * 50 / 2; // reset code duration defaults to 50us
	led_encoder->reset_code = (rmt_symbol_word_t) {
		.level0 = 0,
				.duration0 = reset_ticks,
				.level1 = 0,
				.duration1 = reset_ticks,
	};

	*ret_encoder = &led_encoder->base;
	return ESP_OK;
}

static lbm_value ext_rgbled_deinit(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	if (led_pixels != NULL) {
		free(led_pixels);
		led_pixels = NULL;
	}

	if (led_chan != NULL) {
		rmt_disable(led_chan);
		rmt_del_channel(led_chan);
		led_chan = NULL;
	}

	if (led_encoder != NULL) {
		rmt_del_encoder(led_encoder);
		led_encoder = NULL;
	}

	led_num = -1;

	return ENC_SYM_TRUE;
}

static lbm_value ext_rgbled_init(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	if (!gpio_is_valid(pin)) {
		lbm_set_error_reason(pin_invalid_msg);
		return ENC_SYM_TERROR;
	}

	int num_leds = lbm_dec_as_u32(args[1]);

	if (num_leds == 0) {
		lbm_set_error_reason("At least one led must be used");
		return ENC_SYM_TERROR;
	}

	ext_rgbled_deinit(0, 0);

	led_pixels = calloc(num_leds, 3);

	if (!led_pixels) {
		lbm_set_error_reason("Not enough memory");
		return ENC_SYM_EERROR;
	}

	led_num = num_leds;

	rmt_tx_channel_config_t tx_chan_config = {
			.clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
			.gpio_num = pin,
			.mem_block_symbols = 64, // increase the block size can make the LED less flickering
			.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
			.trans_queue_depth = 4, // set the number of transactions that can be pending in the background
	};
	rmt_new_tx_channel(&tx_chan_config, &led_chan);

	rmt_new_led_strip_encoder(&led_encoder);
	rmt_enable(led_chan);

	return ENC_SYM_TRUE;
}

static lbm_value ext_rgbled_color(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	if (led_encoder == NULL || led_chan == NULL || led_pixels == NULL) {
		lbm_set_error_reason("Please run (rgbled-init pin num-leds) first");
		if (led_encoder == NULL) {
			commands_printf_lisp("led_encoder null");
		}

		if (led_chan == NULL) {
			commands_printf_lisp("led_chan null");
		}
		return ENC_SYM_EERROR;
	}

	int led = lbm_dec_as_u32(args[0]);

	if (led >= led_num) {
		lbm_set_error_reason("Invalid LED number");
		return ENC_SYM_TERROR;
	}

	uint32_t color = lbm_dec_as_u32(args[1]);

	uint8_t r = (color >> 16) & 0xFF;
	uint8_t g = (color >> 8) & 0xFF;
	uint8_t b = color & 0xFF;

	led_pixels[led * 3 + 0] = g;
	led_pixels[led * 3 + 1] = r;
	led_pixels[led * 3 + 2] = b;

	rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config);

	return ENC_SYM_TRUE;
}

// Logging

static lbm_value ext_log_start(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(5);

	if (!lbm_is_number(args[0]) ||
			!lbm_is_number(args[1]) ||
			!lbm_is_number(args[2]) ||
			!is_symbol_true_false(args[3]) ||
			!is_symbol_true_false(args[4])) {
		return ENC_SYM_EERROR;
	}

	log_comm_start(
			lbm_dec_as_i32(args[0]),
			lbm_dec_as_i32(args[1]),
			lbm_dec_as_float(args[2]),
			lbm_is_symbol_true(args[3]),
			lbm_is_symbol_true(args[4]),
			lbm_is_symbol_true(args[4]));

	return ENC_SYM_TRUE;
}

static lbm_value ext_log_stop(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	log_comm_stop(lbm_dec_as_i32(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_log_config_field(lbm_value *args, lbm_uint argn) {
	if (argn != 8) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_EERROR;
	}

	int arg_now = 0;

	int can_id = -1;
	if (lbm_is_number(args[arg_now])) {
		can_id = lbm_dec_as_i32(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	int field_ind = -1;
	if (lbm_is_number(args[arg_now])) {
		field_ind = lbm_dec_as_i32(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	char *key = lbm_dec_str(args[arg_now++]);
	if (key == NULL) {
		return ENC_SYM_EERROR;
	}

	char *name = lbm_dec_str(args[arg_now++]);
	if (name == NULL) {
		return ENC_SYM_EERROR;
	}

	char *unit = lbm_dec_str(args[arg_now++]);
	if (unit == NULL) {
		return ENC_SYM_EERROR;
	}

	int precision = -1;
	if (lbm_is_number(args[arg_now])) {
		precision = lbm_dec_as_i32(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	bool is_relative = false;
	if (is_symbol_true_false(args[arg_now])) {
		is_relative = lbm_is_symbol_true(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	bool is_timestamp = false;
	if (is_symbol_true_false(args[arg_now])) {
		is_timestamp = lbm_is_symbol_true(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	log_comm_config_field(can_id, field_ind, key, name, unit, precision, is_relative, is_timestamp);

	return ENC_SYM_TRUE;
}

static lbm_value log_send_fxx(bool is_64, lbm_value *args, lbm_uint argn) {
	unsigned int arg_now = 0;

	int can_id = -1;
	if (lbm_is_number(args[arg_now])) {
		can_id = lbm_dec_as_i32(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	int field_start = -1;
	if (lbm_is_number(args[arg_now])) {
		field_start = lbm_dec_as_i32(args[arg_now++]);
	} else {
		return ENC_SYM_EERROR;
	}

	int32_t ind = 0;
	uint8_t *buffer = mempools_get_packet_buffer();

	buffer[ind++] = is_64 ? COMM_LOG_DATA_F64 : COMM_LOG_DATA_F32;
	buffer_append_int16(buffer, field_start, &ind);

	int append_cnt = 0;
	int append_max = is_64 ? 50 : 100;

	while (arg_now < argn) {
		if (lbm_is_number(args[arg_now])) {
			if (is_64) {
				buffer_append_float64_auto(buffer, lbm_dec_as_double(args[arg_now]), &ind);
			} else {
				buffer_append_float32_auto(buffer, lbm_dec_as_float(args[arg_now]), &ind);
			}
			append_cnt++;
			if (append_cnt >= append_max) {
				mempools_free_packet_buffer(buffer);
				return ENC_SYM_EERROR;
			}
		} else if (lbm_is_cons(args[arg_now])) {
			lbm_value curr = args[arg_now];
			while (lbm_is_cons(curr)) {
				lbm_value  val = lbm_car(curr);
				if (lbm_is_number(val)) {
					if (is_64) {
						buffer_append_float64_auto(buffer, lbm_dec_as_double(val), &ind);
					} else {
						buffer_append_float32_auto(buffer, lbm_dec_as_float(val), &ind);
					}
					append_cnt++;
					if (append_cnt >= append_max) {
						mempools_free_packet_buffer(buffer);
						return ENC_SYM_EERROR;
					}
				} else {
					mempools_free_packet_buffer(buffer);
					return ENC_SYM_EERROR;
				}

				curr = lbm_cdr(curr);
			}
		} else {
			mempools_free_packet_buffer(buffer);
			return ENC_SYM_EERROR;
		}
		arg_now++;
	}

	log_comm_send(can_id, buffer, ind);

	mempools_free_packet_buffer(buffer);

	return ENC_SYM_TRUE;
}

static lbm_value ext_log_send_f32(lbm_value *args, lbm_uint argn) {
	return log_send_fxx(false, args, argn);
}

static lbm_value ext_log_send_f64(lbm_value *args, lbm_uint argn) {
	return log_send_fxx(true, args, argn);
}

// GNSS

static lbm_value ext_gnss_lat_lon(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	nmea_state_t *s = nmea_get_state();

	lbm_value lat_lon = ENC_SYM_NIL;
	lat_lon = lbm_cons(lbm_enc_double(s->gga.lon), lat_lon);
	lat_lon = lbm_cons(lbm_enc_double(s->gga.lat), lat_lon);

	return lat_lon;
}

static lbm_value ext_gnss_height(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(nmea_get_state()->gga.height);
}

static lbm_value ext_gnss_speed(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(nmea_get_state()->rmc.speed);
}

static lbm_value ext_gnss_hdop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(nmea_get_state()->gga.h_dop);
}

static lbm_value ext_gnss_date_time(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	nmea_state_t *s = nmea_get_state();

	lbm_value lat_lon = ENC_SYM_NIL;
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.ms), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.ss), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.mm), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.hh), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.dd), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.mo), lat_lon);
	lat_lon = lbm_cons(lbm_enc_i(s->rmc.yy), lat_lon);

	return lat_lon;
}

static lbm_value ext_gnss_age(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(UTILS_AGE_S(nmea_get_state()->gga.update_time));
}

static lbm_value ext_sleep_deep(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	esp_bluedroid_disable();
	esp_bt_controller_disable();
	esp_wifi_stop();

	float sleep_time = lbm_dec_as_float(args[0]);
	if (sleep_time > 0) {
		esp_sleep_enable_timer_wakeup((uint32_t)(sleep_time * 1.0e6));
	}

	esp_deep_sleep_start();

	return ENC_SYM_TRUE;
}

static lbm_value ext_sleep_config_wakeup_pin(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	int mode = lbm_dec_as_i32(args[1]);

	if (!gpio_is_valid(pin) || !esp_sleep_is_valid_wakeup_gpio(pin)) {
		lbm_set_error_reason(pin_invalid_msg);
		return ENC_SYM_EERROR;
	}

	gpio_set_direction(pin, GPIO_MODE_INPUT);
	esp_deep_sleep_enable_gpio_wakeup(1 << pin,
			mode ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW);

	return ENC_SYM_TRUE;
}

static lbm_value ext_empty(lbm_value *args, lbm_uint argn) {
	(void)args;(void)argn;
	return ENC_SYM_TRUE;
}

void lispif_load_vesc_extensions(void) {
	if (!i2c_mutex_init_done) {
		i2c_mutex = xSemaphoreCreateMutex();
		i2c_mutex_init_done = true;
	}

	lbm_add_symbol_const("hw-express", &sym_hw_express);
	lbm_add_symbol_const("event-can-sid", &sym_event_can_sid);
	lbm_add_symbol_const("event-can-eid", &sym_event_can_eid);
	lbm_add_symbol_const("event-data-rx", &sym_event_data_rx);
	lbm_add_symbol_const("event-esp-now-rx", &sym_event_esp_now_rx);

	lbm_add_symbol_const("a01", &sym_res);
	lbm_add_symbol_const("a02", &sym_loop);
	lbm_add_symbol_const("break", &sym_break);
	lbm_add_symbol_const("a03", &sym_brk);
	lbm_add_symbol_const("a04", &sym_rst);

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	// Various commands
	lbm_add_extension("print", ext_print);
	lbm_add_extension("get-bms-val", ext_get_bms_val);
	lbm_add_extension("set-bms-val", ext_set_bms_val);
	lbm_add_extension("send-bms-can", ext_send_bms_can);
	lbm_add_extension("get-adc", ext_get_adc);
	lbm_add_extension("systime", ext_systime);
	lbm_add_extension("secs-since", ext_secs_since);
	lbm_add_extension("event-enable", ext_enable_event);
	lbm_add_extension("send-data", ext_send_data);
	lbm_add_extension("sysinfo", ext_sysinfo);
	lbm_add_extension("import", ext_empty);
	lbm_add_extension("main-init-done", ext_main_init_done);
	lbm_add_extension("crc16", ext_crc16);

	// EEPROM
	lbm_add_extension("eeprom-store-f", ext_eeprom_store_f);
	lbm_add_extension("eeprom-read-f", ext_eeprom_read_f);
	lbm_add_extension("eeprom-store-i", ext_eeprom_store_i);
	lbm_add_extension("eeprom-read-i", ext_eeprom_read_i);

	// CAN-comands
	lbm_add_extension("can-scan", ext_can_scan);
	lbm_add_extension("can-send-sid", ext_can_send_sid);
	lbm_add_extension("can-send-eid", ext_can_send_eid);
	lbm_add_extension("can-cmd", ext_can_cmd);
	lbm_add_extension("can-list-devs", ext_can_list_devs);

	lbm_add_extension("canget-current", ext_can_get_current);
	lbm_add_extension("canget-current-dir", ext_can_get_current_dir);
	lbm_add_extension("canget-current-in", ext_can_get_current_in);
	lbm_add_extension("canget-duty", ext_can_get_duty);
	lbm_add_extension("canget-rpm", ext_can_get_rpm);
	lbm_add_extension("canget-temp-fet", ext_can_get_temp_fet);
	lbm_add_extension("canget-temp-motor", ext_can_get_temp_motor);
	lbm_add_extension("canget-speed", ext_can_get_speed);
	lbm_add_extension("canget-dist", ext_can_get_dist);
	lbm_add_extension("canget-ppm", ext_can_get_ppm);
	lbm_add_extension("canget-adc", ext_can_get_adc);
	lbm_add_extension("canget-vin", ext_can_get_vin);

	lbm_add_extension("canset-current", ext_can_current);
	lbm_add_extension("canset-current-rel", ext_can_current_rel);
	lbm_add_extension("canset-duty", ext_can_duty);
	lbm_add_extension("canset-brake", ext_can_brake);
	lbm_add_extension("canset-brake-rel", ext_can_brake_rel);
	lbm_add_extension("canset-rpm", ext_can_rpm);
	lbm_add_extension("canset-pos", ext_can_pos);

	// I2C
	i2c_started = false;
	lbm_add_extension("i2c-start", ext_i2c_start);
	lbm_add_extension("i2c-tx-rx", ext_i2c_tx_rx);

	// GPIO
	lbm_add_extension("gpio-configure", ext_gpio_configure);
	lbm_add_extension("gpio-write", ext_gpio_write);
	lbm_add_extension("gpio-read", ext_gpio_read);

	// Bit operations
	lbm_add_extension("bits-enc-int", ext_bits_enc_int);
	lbm_add_extension("bits-dec-int", ext_bits_dec_int);

	// Macro expanders
	lbm_add_extension("me-defun", ext_me_defun);
	lbm_add_extension("me-loopfor", ext_me_loopfor);
	lbm_add_extension("me-loopwhile", ext_me_loopwhile);
	lbm_add_extension("me-looprange", ext_me_looprange);
	lbm_add_extension("me-loopforeach", ext_me_loopforeach);

	// Lbm settings
	lbm_add_extension("lbm-set-quota", ext_lbm_set_quota);

	// Plot
	lbm_add_extension("plot-init", ext_plot_init);
	lbm_add_extension("plot-add-graph", ext_plot_add_graph);
	lbm_add_extension("plot-set-graph", ext_plot_set_graph);
	lbm_add_extension("plot-send-points", ext_plot_send_points);

	// IO-boards
	lbm_add_extension("ioboard-get-adc", ext_ioboard_get_adc);
	lbm_add_extension("ioboard-get-digital", ext_ioboard_get_digital);
	lbm_add_extension("ioboard-set-digital", ext_ioboard_set_digital);
	lbm_add_extension("ioboard-set-pwm", ext_ioboard_set_pwm);

	// ESP NOW
	lbm_add_extension("esp-now-start", ext_esp_now_start);
	lbm_add_extension("esp-now-add-peer", ext_esp_now_add_peer);
	lbm_add_extension("esp-now-send", ext_esp_now_send);
	lbm_add_extension("get-mac-addr", ext_get_mac_addr);
	lbm_add_extension("wifi-get-chan", ext_wifi_get_chan);
	lbm_add_extension("wifi-set-chan", ext_wifi_set_chan);
	lbm_add_extension("wifi-get-bw", ext_wifi_get_bw);
	lbm_add_extension("wifi-set-bw", ext_wifi_set_bw);

	// RGBLED
	lbm_add_extension("rgbled-init", ext_rgbled_init);
	lbm_add_extension("rgbled-deinit", ext_rgbled_deinit);
	lbm_add_extension("rgbled-color", ext_rgbled_color);

	// Logging
	lbm_add_extension("log-start", ext_log_start);
	lbm_add_extension("log-stop", ext_log_stop);
	lbm_add_extension("log-config-field", ext_log_config_field);
	lbm_add_extension("log-send-f32", ext_log_send_f32);
	lbm_add_extension("log-send-f64", ext_log_send_f64);

	// GNSS
	lbm_add_extension("gnss-lat-lon", ext_gnss_lat_lon);
	lbm_add_extension("gnss-height", ext_gnss_height);
	lbm_add_extension("gnss-speed", ext_gnss_speed);
	lbm_add_extension("gnss-hdop", ext_gnss_hdop);
	lbm_add_extension("gnss-date-time", ext_gnss_date_time);
	lbm_add_extension("gnss-age", ext_gnss_age);

	// Sleep
	lbm_add_extension("sleep-deep", ext_sleep_deep);
	lbm_add_extension("sleep-config-wakeup-pin", ext_sleep_config_wakeup_pin);

        // Disp extensions
	lispif_load_disp_extensions();

	// TODO:
	// - file system
	// - uart?

	// Extension libraries
	lbm_array_extensions_init();
	lbm_string_extensions_init();
	lbm_math_extensions_init();

	if (ext_callback) {
		ext_callback();
	}
}

void lispif_set_ext_load_callback(void (*p_func)(void)) {
	ext_callback = p_func;
}

void lispif_disable_all_events(void) {
	event_can_sid_en = false;
	event_can_eid_en = false;
	event_data_rx_en = false;
}

void lispif_process_can(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	if (!event_can_sid_en && !is_ext) {
		return;
	}

	if (!event_can_eid_en && is_ext) {
		return;
	}

	lbm_flat_value_t v;
	if (lbm_start_flatten(&v, 50 + len)) {
		f_cons(&v);
		f_sym(&v, is_ext ? sym_event_can_eid : sym_event_can_sid);
		f_cons(&v);
		f_i32(&v, can_id);
		f_lbm_array(&v, len, LBM_TYPE_BYTE, data8);
		lbm_finish_flatten(&v);
		if (!lbm_event(&v)) {
			lbm_free(v.buf);
		}
	}
}

void lispif_process_custom_app_data(unsigned char *data, unsigned int len) {
	if (!event_data_rx_en) {
		return;
	}

	lbm_flat_value_t v;
	if (lbm_start_flatten(&v, 30 + len)) {
		f_cons(&v);
		f_sym(&v, sym_event_data_rx);
		f_lbm_array(&v, len, LBM_TYPE_BYTE, data);
		lbm_finish_flatten(&v);
		if (!lbm_event(&v)) {
			lbm_free(v.buf);
		}
	}
}

