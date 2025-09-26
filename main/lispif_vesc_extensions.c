/*
	Copyright 2023, 2024 Benjamin Vedder benjamin@vedder.se
	Copyright 2022, 2024 Joel Svensson   svenssonjoel@yahoo.se
	Copyright 2023 Rasmus Söderhielm     rasmus.soderhielm@gmail.com

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

#include "eval_cps.h"
#include "extensions.h"
#include "heap.h"
#include "lbm_defines.h"
#include "lbm_flat_value.h"
#include "lbm_memory.h"
#include "lbm_types.h"
#include "main.h"
#include "lispif.h"
#include "lispbm.h"
#include "lispif_events.h"
#include "extensions/array_extensions.h"
#include "extensions/string_extensions.h"
#include "extensions/math_extensions.h"
#include "extensions/mutex_extensions.h"
#include "extensions/lbm_dyn_lib.h"
#include "extensions/ttf_extensions.h"
#include "lispif_disp_extensions.h"
#include "lispif_wifi_extensions.h"
#include "lispif_ble_extensions.h"
#include "lispif_rgbled_extensions.h"
#include "lbm_color_extensions.h"
#include "lbm_constants.h"
#include "lbm_vesc_utils.h"
#include "commands.h"
#include "comm_can.h"
#include "conf_general.h"
#include "mempools.h"
#include "log.h"
#include "buffer.h"
#include "nvs.h"
#include "print.h"
#include "utils.h"
#include "rb.h"
#include "crc.h"
#include "bms.h"
#include "nmea.h"
#include "ublox.h"
#include "log_comm.h"
#include "comm_wifi.h"
#include "enc_as504x.h"
#include "imu.h"
#include "comm_usb.h"
#include "comm_uart.h"
#include "comm_ble.h"
#include "lbm_image.h"
#include "packet.h"
#include "flash_helper.h"
#include "crypto.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"
#include "lowzip.h"
#include "aes/esp_aes.h"

#include <math.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

// Declare native lib extension
lbm_value ext_load_native_lib(lbm_value *args, lbm_uint argn);
lbm_value ext_unload_native_lib(lbm_value *args, lbm_uint argn);

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
	lbm_uint pres;
	lbm_uint temp_max_cell;
	lbm_uint v_cell_min;
	lbm_uint v_cell_max;
	lbm_uint soc;
	lbm_uint soh;
	lbm_uint can_id;
	lbm_uint ah_cnt_chg_total;
	lbm_uint wh_cnt_chg_total;
	lbm_uint ah_cnt_dis_total;
	lbm_uint wh_cnt_dis_total;
	lbm_uint msg_age;
	lbm_uint chg_allowed;
	lbm_uint data_version;
	lbm_uint status;

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
	lbm_uint part_running;
	lbm_uint git_branch;
	lbm_uint git_hash;

	// FW Info
	lbm_uint version;
	lbm_uint test_version;
	lbm_uint commit;
	lbm_uint user_commit;

	// Rates
	lbm_uint rate_100k;
	lbm_uint rate_200k;
	lbm_uint rate_400k;
	lbm_uint rate_700k;

	// Settings
	lbm_uint controller_id;
	lbm_uint can_baud_rate;
	lbm_uint can_status_rate_hz;
	lbm_uint wifi_mode;
	lbm_uint wifi_sta_ssid;
	lbm_uint wifi_sta_key;
	lbm_uint wifi_ap_ssid;
	lbm_uint wifi_ap_key;
	lbm_uint use_tcp_local;
	lbm_uint use_tcp_hub;
	lbm_uint tcp_hub_url;
	lbm_uint tcp_hub_port;
	lbm_uint tcp_hub_id;
	lbm_uint tcp_hub_pass;
	lbm_uint ble_mode;
	lbm_uint ble_name;
	lbm_uint ble_pin;
	lbm_uint ble_service_capacity;
	lbm_uint ble_chr_descr_capacity;

	// Arrays
	lbm_uint copy;
	lbm_uint mut;

	// Other
	lbm_uint half_duplex;
} vesc_syms;

static vesc_syms syms_vesc = {0};

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &syms_vesc.v_tot) {
			lbm_add_symbol_const("bms-v-tot", comp);
		} else if (comp == &syms_vesc.v_charge) {
			lbm_add_symbol_const("bms-v-charge", comp);
		} else if (comp == &syms_vesc.i_in) {
			lbm_add_symbol_const("bms-i-in", comp);
		} else if (comp == &syms_vesc.i_in_ic) {
			lbm_add_symbol_const("bms-i-in-ic", comp);
		} else if (comp == &syms_vesc.ah_cnt) {
			lbm_add_symbol_const("bms-ah-cnt", comp);
		} else if (comp == &syms_vesc.wh_cnt) {
			lbm_add_symbol_const("bms-wh-cnt", comp);
		} else if (comp == &syms_vesc.cell_num) {
			lbm_add_symbol_const("bms-cell-num", comp);
		} else if (comp == &syms_vesc.v_cell) {
			lbm_add_symbol_const("bms-v-cell", comp);
		} else if (comp == &syms_vesc.bal_state) {
			lbm_add_symbol_const("bms-bal-state", comp);
		} else if (comp == &syms_vesc.temp_adc_num) {
			lbm_add_symbol_const("bms-temp-adc-num", comp);
		} else if (comp == &syms_vesc.temps_adc) {
			lbm_add_symbol_const("bms-temps-adc", comp);
		} else if (comp == &syms_vesc.temp_ic) {
			lbm_add_symbol_const("bms-temp-ic", comp);
		} else if (comp == &syms_vesc.temp_hum) {
			lbm_add_symbol_const("bms-temp-hum", comp);
		} else if (comp == &syms_vesc.hum) {
			lbm_add_symbol_const("bms-hum", comp);
		} else if (comp == &syms_vesc.pres) {
			lbm_add_symbol_const("bms-pres", comp);
		} else if (comp == &syms_vesc.temp_max_cell) {
			lbm_add_symbol_const("bms-temp-cell-max", comp);
		} else if (comp == &syms_vesc.v_cell_min) {
			lbm_add_symbol_const("bms-v-cell-min", comp);
		} else if (comp == &syms_vesc.v_cell_max) {
			lbm_add_symbol_const("bms-v-cell-max", comp);
		} else if (comp == &syms_vesc.soc) {
			lbm_add_symbol_const("bms-soc", comp);
		} else if (comp == &syms_vesc.soh) {
			lbm_add_symbol_const("bms-soh", comp);
		} else if (comp == &syms_vesc.can_id) {
			lbm_add_symbol_const("bms-can-id", comp);
		} else if (comp == &syms_vesc.ah_cnt_chg_total) {
			lbm_add_symbol_const("bms-ah-cnt-chg-total", comp);
		} else if (comp == &syms_vesc.wh_cnt_chg_total) {
			lbm_add_symbol_const("bms-wh-cnt-chg-total", comp);
		} else if (comp == &syms_vesc.ah_cnt_dis_total) {
			lbm_add_symbol_const("bms-ah-cnt-dis-total", comp);
		} else if (comp == &syms_vesc.wh_cnt_dis_total) {
			lbm_add_symbol_const("bms-wh-cnt-dis-total", comp);
		} else if (comp == &syms_vesc.msg_age) {
			lbm_add_symbol_const("bms-msg-age", comp);
		} else if (comp == &syms_vesc.chg_allowed) {
			lbm_add_symbol_const("bms-chg-allowed", comp);
		} else if (comp == &syms_vesc.data_version) {
			lbm_add_symbol_const("bms-data-version", comp);
		} else if (comp == &syms_vesc.status) {
			lbm_add_symbol_const("bms-status", comp);
		}

		else if (comp == &syms_vesc.pin_mode_out) {
			lbm_add_symbol_const("pin-mode-out", comp);
		} else if (comp == &syms_vesc.pin_mode_od) {
			lbm_add_symbol_const("pin-mode-od", comp);
		} else if (comp == &syms_vesc.pin_mode_od_pu) {
			lbm_add_symbol_const("pin-mode-od-pu", comp);
		} else if (comp == &syms_vesc.pin_mode_od_pd) {
			lbm_add_symbol_const("pin-mode-od-pd", comp);
		} else if (comp == &syms_vesc.pin_mode_in) {
			lbm_add_symbol_const("pin-mode-in", comp);
		} else if (comp == &syms_vesc.pin_mode_in_pu) {
			lbm_add_symbol_const("pin-mode-in-pu", comp);
		} else if (comp == &syms_vesc.pin_mode_in_pd) {
			lbm_add_symbol_const("pin-mode-in-pd", comp);
		} else if (comp == &syms_vesc.pin_mode_analog) {
			lbm_add_symbol_const("pin-mode-analog", comp);
		}

		else if (comp == &syms_vesc.hw_name) {
			lbm_add_symbol_const("hw-name", comp);
		} else if (comp == &syms_vesc.fw_ver) {
			lbm_add_symbol_const("fw-ver", comp);
		} else if (comp == &syms_vesc.uuid) {
			lbm_add_symbol_const("uuid", comp);
		} else if (comp == &syms_vesc.hw_type) {
			lbm_add_symbol_const("hw-type", comp);
		} else if (comp == &syms_vesc.part_running) {
			lbm_add_symbol_const("part-running", comp);
		} else if (comp == &syms_vesc.git_branch) {
			lbm_add_symbol_const("git-branch", comp);
		} else if (comp == &syms_vesc.git_hash) {
			lbm_add_symbol_const("git-hash", comp);
		}

		else if (comp == &syms_vesc.version) {
			lbm_add_symbol_const("version", comp);
		} else if (comp == &syms_vesc.test_version) {
			lbm_add_symbol_const("test-version", comp);
		} else if (comp == &syms_vesc.commit) {
			lbm_add_symbol_const("commit", comp);
		} else if (comp == &syms_vesc.user_commit) {
			lbm_add_symbol_const("user-commit", comp);
		}

		else if (comp == &syms_vesc.rate_100k) {
			lbm_add_symbol_const("rate-100k", comp);
		} else if (comp == &syms_vesc.rate_200k) {
			lbm_add_symbol_const("rate-200k", comp);
		} else if (comp == &syms_vesc.rate_400k) {
			lbm_add_symbol_const("rate-400k", comp);
		} else if (comp == &syms_vesc.rate_700k) {
			lbm_add_symbol_const("rate-700k", comp);
		}

		else if (comp == &syms_vesc.controller_id) {
			lbm_add_symbol_const("controller-id", comp);
		} else if (comp == &syms_vesc.can_baud_rate) {
			lbm_add_symbol_const("can-baud-rate", comp);
		} else if (comp == &syms_vesc.can_status_rate_hz) {
			lbm_add_symbol_const("can-status-rate-hz", comp);
		} else if (comp == &syms_vesc.wifi_mode) {
			lbm_add_symbol_const("wifi-mode", comp);
		} else if (comp == &syms_vesc.wifi_sta_ssid) {
			lbm_add_symbol_const("wifi-sta-ssid", comp);
		} else if (comp == &syms_vesc.wifi_sta_key) {
			lbm_add_symbol_const("wifi-sta-key", comp);
		} else if (comp == &syms_vesc.wifi_ap_ssid) {
			lbm_add_symbol_const("wifi-ap-ssid", comp);
		} else if (comp == &syms_vesc.wifi_ap_key) {
			lbm_add_symbol_const("wifi-ap-key", comp);
		} else if (comp == &syms_vesc.use_tcp_local) {
			lbm_add_symbol_const("use-tcp-local", comp);
		} else if (comp == &syms_vesc.use_tcp_hub) {
			lbm_add_symbol_const("use-tcp-hub", comp);
		} else if (comp == &syms_vesc.tcp_hub_url) {
			lbm_add_symbol_const("tcp-hub-url", comp);
		} else if (comp == &syms_vesc.tcp_hub_port) {
			lbm_add_symbol_const("tcp-hub-port", comp);
		} else if (comp == &syms_vesc.tcp_hub_id) {
			lbm_add_symbol_const("tcp-hub-id", comp);
		} else if (comp == &syms_vesc.tcp_hub_pass) {
			lbm_add_symbol_const("tcp-hub-pass", comp);
		} else if (comp == &syms_vesc.ble_mode) {
			lbm_add_symbol_const("ble-mode", comp);
		} else if (comp == &syms_vesc.ble_name) {
			lbm_add_symbol_const("ble-name", comp);
		} else if (comp == &syms_vesc.ble_pin) {
			lbm_add_symbol_const("ble-pin", comp);
		} else if (comp == &syms_vesc.ble_service_capacity) {
			lbm_add_symbol_const("ble-service-capacity", comp);
		} else if (comp == &syms_vesc.ble_chr_descr_capacity) {
			lbm_add_symbol_const("ble-chr-descr-capacity", comp);
		}

		else if (comp == &syms_vesc.copy) {
			lbm_add_symbol_const("copy", comp);
		} else if (comp == &syms_vesc.mut) {
			lbm_add_symbol_const("mut", comp);
		}

		else if (comp == &syms_vesc.half_duplex) {
			lbm_add_symbol_const("half-duplex", comp);
		}
	}

	return *comp == sym;
}

static bool start_flatten_with_gc(lbm_flat_value_t *v, size_t buffer_size) {
	if (lispif_is_eval_task()) {
		return lbm_start_flatten(v, buffer_size);
	}

	if (lbm_start_flatten(v, buffer_size)) {
		return true;
	}

	int timeout = 3;
	uint32_t gc_last = lbm_heap_state.gc_num;
	lbm_request_gc();

	while (lbm_heap_state.gc_num <= gc_last && timeout > 0) {
		vTaskDelay(1);
		timeout--;
	}

	return lbm_start_flatten(v, buffer_size);
}

static bool is_symbol_true_false(lbm_value v) {
	bool res = lbm_is_symbol_true(v) || lbm_is_symbol_nil(v);
	lbm_set_error_reason("Argument must be t or nil (true or false)");
	return res;
}

// Various commands

static lbm_value ext_print(lbm_value *args, lbm_uint argn) {
	const int str_len = 256;
	char *print_val_buffer = lbm_malloc_reserve(str_len);
	if (!print_val_buffer) {
		return ENC_SYM_MERROR;
	}

	for (lbm_uint i = 0; i < argn; i ++) {
		lbm_print_value(print_val_buffer, str_len, args[i]);
		commands_printf_lisp("%s", print_val_buffer);
	}

	lbm_free(print_val_buffer);

	return ENC_SYM_TRUE;
}

static char print_prefix[50] = {0};
static char fw_name[20] = {0};

static lbm_value ext_set_print_prefix(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	if (!lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	const char *string = lbm_dec_str(args[0]);
	strncpy(print_prefix, string, sizeof(print_prefix) - 1);

	return ENC_SYM_TRUE;
}

static lbm_value ext_set_fw_name(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	if (!lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	const char *string = lbm_dec_str(args[0]);
	strncpy(fw_name, string, sizeof(fw_name) - 1);

	return ENC_SYM_TRUE;
}

/**
 * signature: (puts string)
 *
 * Print string without surrounding it with "quotes" first.
 *
 * @param string The string to print. Strings longer than 400 characters will be
 * trimmed.
*/
static lbm_value ext_puts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	if (!lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	const char *string = lbm_dec_str(args[0]);
	commands_printf_lisp("%s", string);

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

static lbm_value get_or_set_u16(bool set, uint16_t *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_i(*val);
	}
}

static lbm_value get_or_set_u32(bool set, uint32_t *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_u32(*lbm_val);
		return ENC_SYM_TRUE;
	} else {
		return lbm_enc_u32(*val);
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

static lbm_value get_or_set_string(bool set, char *val, lbm_value *lbm_val, int max_len) {
	if (set) {
		char *str = lbm_dec_str(*lbm_val);
		if (str) {
			strncpy(val, str, max_len - 1);
			val[max_len - 1] = '\0';
			return ENC_SYM_TRUE;
		} else {
			return ENC_SYM_TERROR;
		}
	} else {
		lbm_value res;
		lbm_uint len = strnlen(val, max_len);
		if (lbm_create_array(&res, len + 1)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);
			memcpy(arr->data, val, len);
			((char*)(arr->data))[len] = '\0';
			return res;
		} else {
			return ENC_SYM_MERROR;
		}
	}
}

static lbm_value get_set_bms_val(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name = lbm_dec_sym(args[0]);
	bms_values *val = (bms_values*)bms_get_values();

	if (set && !compare_symbol(name, &syms_vesc.status) && !lbm_is_number(set_arg)) {
		lbm_set_error_reason((char*) lbm_error_str_no_number);
		return ENC_SYM_TERROR;
	}

	if (compare_symbol(name, &syms_vesc.v_tot)) {
		res = get_or_set_float(set, &val->v_tot, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_charge)) {
		res = get_or_set_float(set, &val->v_charge, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.i_in)) {
		res = get_or_set_float(set, &val->i_in, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.i_in_ic)) {
		res = get_or_set_float(set, &val->i_in_ic, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.ah_cnt)) {
		res = get_or_set_float(set, &val->ah_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.wh_cnt)) {
		res = get_or_set_float(set, &val->wh_cnt, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.cell_num)) {
		if (set && lbm_dec_as_i32(set_arg) >= BMS_MAX_CELLS) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_i(set, &val->cell_num, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_cell)) {
		if (argn != 2 || !lbm_is_number(args[1])) {
			return ENC_SYM_TERROR;
		}

		int c = lbm_dec_as_i32(args[1]);
		if (c < 0 || c >= val->cell_num) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_float(set, &val->v_cell[c], &set_arg);
	} else if (compare_symbol(name, &syms_vesc.bal_state)) {
		if (argn != 2 || !lbm_is_number(args[1])) {
			return ENC_SYM_TERROR;
		}

		int c = lbm_dec_as_i32(args[1]);
		if (c < 0 || c >= val->cell_num) {
			return ENC_SYM_EERROR;
		}

		res = get_or_set_bool(set, &val->bal_state[c], &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_adc_num)) {
		if (set && lbm_dec_as_i32(set_arg) >= BMS_MAX_TEMPS) {
			return ENC_SYM_EERROR;
		}

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
	} else if (compare_symbol(name, &syms_vesc.pres)) {
		res = get_or_set_float(set, &val->pressure, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.temp_max_cell)) {
		res = get_or_set_float(set, &val->temp_max_cell, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_cell_min)) {
		res = get_or_set_float(set, &val->v_cell_min, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.v_cell_max)) {
		res = get_or_set_float(set, &val->v_cell_max, &set_arg);
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
	} else if (compare_symbol(name, &syms_vesc.chg_allowed)) {
		res = get_or_set_i(set, &val->is_charge_allowed, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.data_version)) {
		res = get_or_set_i(set, &val->data_version, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.status)) {
		res = get_or_set_string(set, val->status, &set_arg, BMS_STATUS_LEN);
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

static lbm_value ext_set_bms_chg_allowed(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int allowed = lbm_dec_as_i32(args[0]);

	uint8_t data[2];
	data[0] = COMM_BMS_SET_CHARGE_ALLOWED;
	data[1] = allowed;

	bms_process_cmd(data, 2, 0);

	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_force_balance(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int force = lbm_dec_as_i32(args[0]);

	uint8_t data[2];
	data[0] = COMM_BMS_FORCE_BALANCE;
	data[1] = force;

	bms_process_cmd(data, 2, 0);

	return ENC_SYM_TRUE;
}

static lbm_value ext_bms_zero_offset(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	uint8_t data[1];
	data[0] = COMM_BMS_ZERO_CURRENT_OFFSET;
	bms_process_cmd(data, 1, 0);

	return ENC_SYM_TRUE;
}

static lbm_value ext_conf_setget(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;

	lbm_value set_arg = 0;
	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg) && !lbm_is_array_r(set_arg)) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}
	}

	if (argn != 1 && argn != 2) {
		return res;
	}

	if (lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	main_config_t *conf = (main_config_t*)&backup.config;
	lbm_uint name = lbm_dec_sym(args[0]);

	if (compare_symbol(name, &syms_vesc.controller_id)) {
		res = get_or_set_i(set, &conf->controller_id, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.can_baud_rate)) {
		int v = conf->can_baud_rate;
		res = get_or_set_i(set, &v, &set_arg);
		if (v != conf->can_baud_rate) {
			conf->can_baud_rate = v;
			comm_can_update_baudrate(0);
		}
	} else if (compare_symbol(name, &syms_vesc.can_status_rate_hz)) {
		res = get_or_set_i(set, &conf->can_status_rate_hz, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.wifi_mode)) {
		int v = conf->wifi_mode;
		res = get_or_set_i(set, &v, &set_arg);
		conf->wifi_mode = v;
	} else if (compare_symbol(name, &syms_vesc.wifi_sta_ssid)) {
		res = get_or_set_string(set, conf->wifi_sta_ssid, &set_arg, sizeof(conf->wifi_sta_ssid));
	} else if (compare_symbol(name, &syms_vesc.wifi_sta_key)) {
		res = get_or_set_string(set, conf->wifi_sta_key, &set_arg, sizeof(conf->wifi_sta_key));
	} else if (compare_symbol(name, &syms_vesc.wifi_ap_ssid)) {
		res = get_or_set_string(set, conf->wifi_ap_ssid, &set_arg, sizeof(conf->wifi_ap_ssid));
	} else if (compare_symbol(name, &syms_vesc.wifi_ap_key)) {
		res = get_or_set_string(set, conf->wifi_ap_key, &set_arg, sizeof(conf->wifi_ap_key));
	} else if (compare_symbol(name, &syms_vesc.use_tcp_local)) {
		res = get_or_set_bool(set, &conf->use_tcp_local, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.use_tcp_hub)) {
		res = get_or_set_bool(set, &conf->use_tcp_hub, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.tcp_hub_url)) {
		res = get_or_set_string(set, conf->tcp_hub_url, &set_arg, sizeof(conf->tcp_hub_url));
	} else if (compare_symbol(name, &syms_vesc.tcp_hub_port)) {
		res = get_or_set_u16(set, &conf->tcp_hub_port, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.tcp_hub_id)) {
		res = get_or_set_string(set, conf->tcp_hub_id, &set_arg, sizeof(conf->tcp_hub_id));
	} else if (compare_symbol(name, &syms_vesc.tcp_hub_pass)) {
		res = get_or_set_string(set, conf->tcp_hub_pass, &set_arg, sizeof(conf->tcp_hub_pass));
	} else if (compare_symbol(name, &syms_vesc.ble_mode)) {
		int v = conf->ble_mode;
		res = get_or_set_i(set, &v, &set_arg);
		conf->ble_mode = v;
	} else if (compare_symbol(name, &syms_vesc.ble_name)) {
		res = get_or_set_string(set, conf->ble_name, &set_arg, sizeof(conf->ble_name));
	} else if (compare_symbol(name, &syms_vesc.ble_pin)) {
		res = get_or_set_u32(set, &conf->ble_pin, &set_arg);
	} else if (compare_symbol(name, &syms_vesc.ble_service_capacity)) {
		int v = conf->ble_service_capacity;
		res = get_or_set_i(set, &v, &set_arg);
		conf->ble_service_capacity = v;
	} else if (compare_symbol(name, &syms_vesc.ble_chr_descr_capacity)) {
		int v = conf->ble_chr_descr_capacity;
		res = get_or_set_i(set, &v, &set_arg);
		conf->ble_chr_descr_capacity = v;
	}

	return res;
}

static lbm_value ext_conf_get(lbm_value *args, lbm_uint argn) {
	return ext_conf_setget(false, args, argn);
}

static lbm_value ext_conf_set(lbm_value *args, lbm_uint argn) {
	return ext_conf_setget(true, args, argn);
}

static lbm_value ext_conf_store(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

static lbm_value ext_reboot(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	comm_wifi_disconnect();
	vTaskDelay(50 / portTICK_PERIOD_MS);
	esp_restart();
	return ENC_SYM_TRUE;
}

static lbm_value ext_get_adc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (argn == 0) {
		return lbm_enc_float(adc_get_voltage(HW_ADC_CH0));
	} else if (argn == 1) {
		lbm_int channel = lbm_dec_as_i32(args[0]);
		if (channel == 0) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH0));
		} else if (channel == 1) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH1));
		} else if (channel == 2) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH2));
		} else if (channel == 3) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH3));
		} else if (channel == 4) {
			return lbm_enc_float(adc_get_voltage(HW_ADC_CH4));
		} else {
			return ENC_SYM_EERROR;
		}
	} else {
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_systime(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_u32(xTaskGetTickCount());
}

static lbm_value ext_secs_since(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	return lbm_enc_float(UTILS_AGE_S(lbm_dec_as_u32(args[0])));
}

static void send_app_data(unsigned char *data, unsigned int len, int interface, int can_id) {
	int32_t index = 0;
	uint8_t *send_buffer_global = mempools_get_lbm_packet_buffer();
	send_buffer_global[index++] = COMM_CUSTOM_APP_DATA;
	memcpy(send_buffer_global + index, data, len);
	index += len;

	switch (interface) {
	case 0:
		commands_send_packet(send_buffer_global, index);
		break;

	case 1:
		comm_usb_send_packet(send_buffer_global, index);
		break;

	case 2:
		comm_can_send_buffer(can_id, send_buffer_global, index, 3);
		break;

	case 3:
		comm_uart_send_packet(send_buffer_global, index, 0);
		break;

	case 4:
		comm_uart_send_packet(send_buffer_global, index, 1);
		break;

	case 5:
		break;

	case 6:
		comm_wifi_send_packet_local(send_buffer_global, index);
		break;

	case 7:
		comm_wifi_send_packet_hub(send_buffer_global, index);
		break;

	case 8:
		comm_ble_send_packet(send_buffer_global, index);
		break;

	default:
		break;
	}


	mempools_free_packet_buffer(send_buffer_global);
}

static lbm_value ext_send_data(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && argn != 2 && argn != 3) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	if (!lbm_is_cons(args[0]) && !lbm_is_array_r(args[0])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	int interface = 0;
	if (argn >= 2) {
		if (!lbm_is_number(args[1])) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}

		interface = lbm_dec_as_i32(args[1]);
	}

	int can_id = 0;
	if (argn >= 3) {
		if (!lbm_is_number(args[2])) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}

		can_id = lbm_dec_as_i32(args[2]);
	}

	lbm_value curr = args[0];
	const int max_len = 100;
	uint8_t to_send[max_len];
	uint8_t *to_send_ptr = to_send;
	int ind = 0;

	if (lbm_is_array_r(args[0])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
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

	send_app_data(to_send_ptr, ind, interface, can_id);

	return ENC_SYM_TRUE;
}

static volatile lbm_cid recv_data_cid = -1;

static lbm_value ext_recv_data(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
	}

	float timeout = -1.0;
	if (argn == 1) {
		timeout = lbm_dec_as_float(args[0]);
	}

	recv_data_cid = lbm_get_current_cid();

	if (timeout > 0.0) {
		lbm_block_ctx_from_extension_timeout(timeout);
	} else {
		lbm_block_ctx_from_extension();
	}

	return ENC_SYM_TRUE;
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

static lbm_value ext_eeprom_erase(lbm_value *args, lbm_uint argn){
	LBM_CHECK_ARGN_NUMBER(1);

	int addr = lbm_dec_as_i32(args[0]);
	if (!check_eeprom_addr(addr)) {
		return ENC_SYM_EERROR;
	}

	char key[10];
	sprintf(key, "v%d", addr);

	nvs_handle_t nvs_handle;
	esp_err_t ok_op = nvs_open("lbm", NVS_READWRITE, &nvs_handle);
	if (ok_op != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	esp_err_t ok_set = nvs_erase_key(nvs_handle, key);
	esp_err_t ok_com = nvs_commit(nvs_handle);
	nvs_close(nvs_handle);

	if (ok_set != ESP_OK || ok_com != ESP_OK) {
		return ENC_SYM_EERROR;
	}
	return ENC_SYM_TRUE;
}

static lbm_uint sym_hw_express;
static lbm_uint sym_size;

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
		if (lbm_create_array(&lbm_res, strlen(HW_NAME) + 1)) {
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
	} else if (compare_symbol(name, &syms_vesc.part_running)) {
		const esp_partition_t *running = esp_ota_get_running_partition();
		if (running != NULL) {
			lbm_value lbm_res;
			if (lbm_create_array(&lbm_res, strlen(running->label) + 1)) {
				lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(lbm_res);
				strcpy((char*)arr->data, running->label);
				res = lbm_res;
			} else {
				res = ENC_SYM_MERROR;
			}
		}
	} else if (compare_symbol(name, &syms_vesc.git_branch)) {
		lbm_value lbm_res;
		if (lbm_create_array(&lbm_res, strlen(GIT_BRANCH_NAME) + 1)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(lbm_res);
			strcpy((char*)arr->data, GIT_BRANCH_NAME);
			res = lbm_res;
		} else {
			res = ENC_SYM_MERROR;
		}
	} else if (compare_symbol(name, &syms_vesc.git_hash)) {
		lbm_value lbm_res;
		if (lbm_create_array(&lbm_res, strlen(GIT_COMMIT_HASH) + 1)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(lbm_res);
			strcpy((char*)arr->data, GIT_COMMIT_HASH);
			res = lbm_res;
		} else {
			res = ENC_SYM_MERROR;
		}
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

	uint8_t *send_buf = mempools_get_lbm_packet_buffer();
	send_buf[0] = COMM_LISP_REPL_CMD;
	memcpy(send_buf + 1, array->data, array->size);
	comm_can_send_buffer(id, send_buf, array->size + 1, 2);
	mempools_free_packet_buffer(send_buf);

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_msg_age(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int id = lbm_dec_as_i32(args[0]);
	int msg = lbm_dec_as_i32(args[1]);

	if (id < 0 || id > 253) {
		return ENC_SYM_EERROR;
	}

	switch (msg) {
	case 1: {
		can_status_msg *stat = comm_can_get_status_msg_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	case 2: {
		can_status_msg_2 *stat = comm_can_get_status_msg_2_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	case 3: {
		can_status_msg_3 *stat = comm_can_get_status_msg_3_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	case 4: {
		can_status_msg_4 *stat = comm_can_get_status_msg_4_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	case 5: {
		can_status_msg_5 *stat = comm_can_get_status_msg_5_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	case 6: {
		can_status_msg_6 *stat = comm_can_get_status_msg_6_id(lbm_dec_as_i32(args[0]));
		if (stat) {
			return lbm_enc_float(UTILS_AGE_S(stat->rx_time));
		} else {
			return ENC_SYM_NIL;
		}
	}

	default:
		return ENC_SYM_EERROR;
	}
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

static lbm_value ext_can_local_id(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i(backup.config.controller_id);
}

static lbm_value ext_can_update_baud(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);
	int kbits = lbm_dec_as_i32(args[0]);
	CAN_BAUD baud = comm_can_kbits_to_baud(kbits);
	if (baud != CAN_BAUD_INVALID) {
		for (int i = 0;i < 10;i++) {
			comm_can_send_update_baud(kbits, 1000);
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
		backup.config.can_baud_rate = baud;

		main_store_backup_data();
		comm_can_update_baudrate(1000);
		return ENC_SYM_TRUE;
	} else {
		return ENC_SYM_TERROR;
	}
}

static lbm_value ext_can_start(lbm_value *args, lbm_uint argn) {
	if (argn > 2) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

#ifdef CAN_TX_GPIO_NUM
	int pin_tx = CAN_TX_GPIO_NUM;
#else
	int pin_tx = -1;
#endif
	if (argn >= 1) {
		pin_tx = lbm_dec_as_u32(args[0]);
	}

#ifdef CAN_RX_GPIO_NUM
	int pin_rx = CAN_RX_GPIO_NUM;
#else
	int pin_rx = -1;
#endif
	if (argn >= 2) {
		pin_rx = lbm_dec_as_u32(args[1]);
	}

	if (!utils_gpio_is_valid(pin_tx) && !utils_gpio_is_valid(pin_rx)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	comm_can_start(pin_tx, pin_rx);

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_stop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	comm_can_stop();
	return ENC_SYM_TRUE;
}

static lbm_value ext_can_use_vesc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	if (!is_symbol_true_false(args[0])) {
		return ENC_SYM_TERROR;
	}

	comm_can_use_vesc_decoder(lbm_is_symbol_true(args[0]));

	return ENC_SYM_TRUE;
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

static lbm_value ext_can_ping(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int id = lbm_dec_as_i32(args[0]);
	if (id < 0 || id > 253) {
		return ENC_SYM_TERROR;
	}

	HW_TYPE hw = HW_TYPE_VESC;
	bool res = comm_can_ping(id, &hw);

	return res ? lbm_enc_i(hw) : ENC_SYM_NIL;
}

static lbm_value ext_can_send(lbm_value *args, lbm_uint argn, bool is_eid) {
	if (argn != 2 || !lbm_is_number(args[0])) {
		return ENC_SYM_EERROR;
	}

	lbm_value curr = args[1];
	uint8_t to_send[8];
	int ind = 0;

	if (lbm_is_array_r(curr)) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(curr);
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

static volatile lbm_cid can_recv_sid_cid = -1;
static volatile lbm_cid can_recv_eid_cid = -1;

static lbm_value ext_can_recv_sid(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
	}

	float timeout = -1.0;
	if (argn == 1) {
		timeout = lbm_dec_as_float(args[0]);
	}

	can_recv_sid_cid = lbm_get_current_cid();

	if (timeout > 0.0) {
		lbm_block_ctx_from_extension_timeout(timeout);
	} else {
		lbm_block_ctx_from_extension();
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_recv_eid(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
	}

	float timeout = -1.0;
	if (argn == 1) {
		timeout = lbm_dec_as_float(args[0]);
	}

	can_recv_eid_cid = lbm_get_current_cid();

	if (timeout > 0.0) {
		lbm_block_ctx_from_extension_timeout(timeout);
	} else {
		lbm_block_ctx_from_extension();
	}

	return ENC_SYM_TRUE;
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

// Math

static lbm_value ext_throttle_curve(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);
	return lbm_enc_float(utils_throttle_curve(
			lbm_dec_as_float(args[0]),
			lbm_dec_as_float(args[1]),
			lbm_dec_as_float(args[2]),
			lbm_dec_as_i32(args[3])));
}

static lbm_value ext_rand(lbm_value *args, lbm_uint argn) {
	if (argn != 0 && argn != 1) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	unsigned int seed = 0;
	bool seed_set = false;

	if (argn == 1) {
		if (!lbm_is_number(args[0])) {
			lbm_set_error_reason((char*)lbm_error_str_no_number);
			return ENC_SYM_TERROR;
		}

		seed = lbm_dec_as_u32(args[0]);
		seed_set = true;
	}

	if (seed_set) {
		srand(seed);
	}

	return lbm_enc_i32(rand());
}

static lbm_value ext_rand_max(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i32(RAND_MAX);
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

static void bms_cmd_handler(COMM_PACKET_ID cmd, int param1, int param2) {
	switch (cmd) {
	case COMM_BMS_SET_CHARGE_ALLOWED: {
		if (event_bms_chg_allow_en) {
			lbm_flat_value_t v;
			if (start_flatten_with_gc(&v, 50)) {
				f_cons(&v);
				f_sym(&v, sym_bms_chg_allow);
				f_cons(&v);
				f_i(&v, param1);
				f_sym(&v, ENC_SYM_NIL);

				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}
	} break;

	case COMM_BMS_SET_BALANCE_OVERRIDE: {
		if (event_bms_bal_ovr_en) {
			lbm_flat_value_t v;
			if (start_flatten_with_gc(&v, 50)) {
				f_cons(&v);
				f_sym(&v, sym_bms_bal_ovr);
				f_cons(&v);
				f_i(&v, param1);
				f_cons(&v);
				f_i(&v, param2);
				f_sym(&v, ENC_SYM_NIL);

				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}
	} break;

	case COMM_BMS_RESET_COUNTERS: {
		if (event_bms_reset_cnt_en) {
			lbm_flat_value_t v;
			if (start_flatten_with_gc(&v, 50)) {
				f_cons(&v);
				f_sym(&v, sym_bms_reset_cnt);
				f_cons(&v);
				f_i(&v, param1);
				f_cons(&v);
				f_i(&v, param2);
				f_sym(&v, ENC_SYM_NIL);

				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}
	} break;

	case COMM_BMS_FORCE_BALANCE: {
		if (event_bms_force_bal_en) {
			lbm_flat_value_t v;
			if (start_flatten_with_gc(&v, 50)) {
				f_cons(&v);
				f_sym(&v, sym_bms_force_bal);
				f_cons(&v);
				f_i(&v, param1);
				f_sym(&v, ENC_SYM_NIL);

				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}
	} break;

	case COMM_BMS_ZERO_CURRENT_OFFSET: {
		if (event_bms_zero_ofs_en) {
			lbm_flat_value_t v;
			if (start_flatten_with_gc(&v, 50)) {
				f_sym(&v, sym_bms_zero_ofs);

				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}
	} break;

	default:
		break;
	}
}

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
	} else if (name == sym_event_ble_rx) {
		event_ble_rx_en = en;
	} else if (name == sym_event_wifi_disconnect) {
		event_wifi_disconnect_en = en;
	} else if (name == sym_event_cmds_data_tx) {
		event_cmds_data_tx_en = en;
	} else if (name == sym_bms_chg_allow) {
		event_bms_chg_allow_en = en;
	} else if (name == sym_bms_bal_ovr) {
		event_bms_bal_ovr_en = en;
	} else if (name == sym_bms_reset_cnt) {
		event_bms_reset_cnt_en = en;
	} else if (name == sym_bms_force_bal) {
		event_bms_force_bal_en = en;
	} else if (name == sym_bms_zero_ofs) {
		event_bms_zero_ofs_en = en;
	} else {
		return ENC_SYM_EERROR;
	}

	if (event_bms_chg_allow_en || event_bms_bal_ovr_en ||
			event_bms_reset_cnt_en || event_bms_force_bal_en || event_bms_zero_ofs_en) {
		bms_register_cmd_handler(bms_cmd_handler);
	} else {
		bms_register_cmd_handler(NULL);
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_lbm_set_quota(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);
	uint32_t q = lbm_dec_as_u32(args[0]);

	if (q < 1) {
		return ENC_SYM_EERROR;
	}

#ifdef LBM_USE_TIME_QUOTA
	lbm_set_eval_time_quota(q);
#else
	lbm_set_eval_step_quota(q);
#endif

	return ENC_SYM_TRUE;
}

lbm_value ext_lbm_set_gc_stack_size(lbm_value *args, lbm_uint argn) {
	if (argn == 1) {
		if (lbm_is_number(args[0])) {
			uint32_t n = lbm_dec_as_u32(args[0]);
			lbm_uint *new_stack = lbm_malloc(n * sizeof(lbm_uint));
			if (new_stack) {
				lbm_free(lbm_heap_state.gc_stack.data);
				lbm_heap_state.gc_stack.data = new_stack;
				lbm_heap_state.gc_stack.size = n;
				lbm_heap_state.gc_stack.sp = 0;
				return ENC_SYM_TRUE;
			}
			return ENC_SYM_MERROR;
		}
	}
	return ENC_SYM_TERROR;
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
static volatile lbm_cid esp_now_send_cid = -1;
static volatile lbm_cid esp_now_recv_cid = -1;
static char *esp_init_msg = "ESP-NOW not initialized";

typedef struct {
	uint8_t *data;
	int len;
	uint8_t src[6];
	uint8_t des[6];
	int rssi;
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
		if (start_flatten_with_gc(&v, 150 + data.len)) {
			if (esp_now_recv_cid < 0) {
				f_cons(&v);
				f_sym(&v, sym_event_esp_now_rx);
			}

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
			f_lbm_array(&v, data.len, data.data);

			f_cons(&v);
			f_i(&v, data.rssi);

			f_sym(&v, SYM_NIL);

			lbm_finish_flatten(&v);

			if (esp_now_recv_cid >= 0) {
				if (!lbm_unblock_ctx(esp_now_recv_cid, &v)) {
					lbm_free(v.buf);
				}
			} else {
				if (!lbm_event(&v)) {
					lbm_free(v.buf);
				}
			}
		}

		free(data.data);
	}
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
#else
static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
#endif
	lbm_unblock_ctx_unboxed(esp_now_send_cid, status == ESP_NOW_SEND_SUCCESS ? ENC_SYM_TRUE : ENC_SYM_NIL);
}

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
	if (event_esp_now_rx_en || esp_now_recv_cid >= 0) {
		esp_now_send_data sdata;

		sdata.data = malloc(data_len);
		if (!sdata.data) {
			return;
		}

		sdata.len = data_len;
		memcpy(sdata.data, data, data_len);
		memcpy(sdata.src, esp_now_info->src_addr, 6);
		memcpy(sdata.des, esp_now_info->des_addr, 6);
		sdata.rssi = esp_now_info->rx_ctrl->rssi;

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
		esp_wifi_set_mode(WIFI_MODE_APSTA);

		if (backup.config.ble_mode == BLE_MODE_DISABLED) {
			esp_wifi_set_ps(WIFI_PS_NONE);
		}

		// The event handler allows some of the wifi-extensions
		// to work.
		esp_event_handler_instance_t instance_any_id;
		esp_event_handler_instance_register(
				WIFI_EVENT,
				ESP_EVENT_ANY_ID,
				&comm_wifi_event_handler,
				NULL,
				&instance_any_id);

		// Enable FTM responder
		wifi_config_t wifi_config;
		memset(&wifi_config, 0, sizeof(wifi_config));
		esp_wifi_get_config(WIFI_IF_AP, &wifi_config);
		wifi_config.ap.ftm_responder = true;
		esp_wifi_set_config(WIFI_IF_AP, &wifi_config);

		esp_wifi_start();
	}

	if (!esp_now_initialized) {
		if (esp_now_init() != ESP_OK) {
			return ENC_SYM_EERROR;
		}

		esp_now_rx_sem = xSemaphoreCreateBinary();
		rb_init(&esp_now_rx_rb, esp_now_rx_data, sizeof(esp_now_send_data), ESP_NOW_RX_BUFFER_ELEMENTS);
		xTaskCreate(esp_rx_fun, "esp_rx", 2048, NULL, 3, NULL);
		esp_now_recv_cid = -1;

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

	if ((argn != 1 && argn != 2) || !lbm_is_list(args[0])) {
		lbm_set_error_reason(lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	int rate = -1;
	if (argn >= 2) {
		if (!lbm_is_number(args[1]) || lbm_dec_as_i32(args[2]) > 15) {
			lbm_set_error_reason(lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}

		rate = lbm_dec_as_i32(args[2]);
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

	if (rate >= 0) {
		esp_now_rate_config_t rate_cfg;
		rate_cfg.phymode = WIFI_PHY_MODE_HT20;
		rate_cfg.dcm = false;
		rate_cfg.ersu = false;
		rate_cfg.rate = rate;
		esp_now_set_peer_rate_config(addr, &rate_cfg);
	}

	if (res == ESP_OK || res == ESP_ERR_ESPNOW_EXIST) {
		return ENC_SYM_TRUE;
	} else {
		return ENC_SYM_EERROR;
	}
}

static lbm_value ext_esp_now_del_peer(lbm_value *args, lbm_uint argn) {
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

	esp_err_t res = esp_now_del_peer(addr);

	if (res == ESP_OK || res == ESP_ERR_ESPNOW_NOT_FOUND) {
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

	// Change country code so that all channels can be used
	wifi_country_t country;
	country.cc[0] = 'J';
	country.cc[1] = 'P';
	country.cc[2] = '\0';
	country.schan = 1;
	country.nchan = 14;
	country.policy = WIFI_COUNTRY_POLICY_MANUAL;

	esp_wifi_set_country(&country);

	esp_err_t res = esp_wifi_set_channel(ch, 0);

	if (res == ESP_ERR_WIFI_NOT_INIT) {
		lbm_set_error_reason(str_wifi_not_init_msg);
		return ENC_SYM_EERROR;
	}

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

static lbm_value ext_wifi_start(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	esp_wifi_start();
	return ENC_SYM_TRUE;
}

static lbm_value ext_wifi_stop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	esp_wifi_stop();
	return ENC_SYM_TRUE;
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

static lbm_value ext_esp_now_recv(lbm_value *args, lbm_uint argn) {
	if (!esp_now_initialized) {
		lbm_set_error_reason(esp_init_msg);
		return ENC_SYM_EERROR;
	}

	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
	}

	float timeout = -1.0;
	if (argn == 1) {
		timeout = lbm_dec_as_float(args[0]);
	}

	esp_now_recv_cid = lbm_get_current_cid();

	if (timeout > 0.0) {
		lbm_block_ctx_from_extension_timeout(timeout);
	} else {
		lbm_block_ctx_from_extension();
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
	if (read_size > 0 && read_buffer != NULL) {
		if (write_size > 0 && write_buffer != NULL) {
			res = i2c_master_write_read_device(0, addr, write_buffer, write_size, read_buffer, read_size, 2000);
		} else {
			res = i2c_master_read_from_device(0, addr, read_buffer, read_size, 2000);
		}
	} else {
		res = i2c_master_write_to_device(0, addr, write_buffer, write_size, 2000);
	}
	xSemaphoreGive(i2c_mutex);

	return res;
}

static char *i2c_not_started_msg = "I2C not started";

static lbm_value ext_i2c_tx_rx(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	if (!i2c_started) {
		lbm_set_error_reason(i2c_not_started_msg);
		return ENC_SYM_EERROR;
	}

	uint16_t addr = 0;
	size_t txlen = 0;
	size_t rxlen = 0;
	uint8_t *txbuf = 0;
	uint8_t *rxbuf = 0;
	bool is_arr = lbm_is_array_r(args[1]);

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}
	addr = lbm_dec_as_u32(args[0]);

	if (is_arr) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
		txbuf = (uint8_t*)array->data;
		txlen = array->size;
	} else {
		txlen = lbm_list_length(args[1]);

		if (txlen > 0) {
			txbuf = lbm_malloc(txlen);
			if (!txbuf) {
				return ENC_SYM_MERROR;
			}

			lbm_value curr = args[1];
			int ind = 0;
			while (lbm_is_cons(curr)) {
				lbm_value  arg = lbm_car(curr);

				if (lbm_is_number(arg)) {
					txbuf[ind++] = lbm_dec_as_u32(arg);
				} else {
					lbm_free(txbuf);
					return ENC_SYM_TERROR;
				}

				curr = lbm_cdr(curr);
			}
		}
	}

	if (argn >= 3 && lbm_is_array_rw(args[2])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);
		rxbuf = (uint8_t*)array->data;
		rxlen = array->size;
	}

	esp_err_t res = i2c_tx_rx(addr, txbuf, txlen, rxbuf, rxlen);

	if (!is_arr && txbuf) {
		lbm_free(txbuf);
	}

	return lbm_enc_i(res);
}

static lbm_value ext_i2c_detect_addr(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	if (!i2c_started) {
		lbm_set_error_reason(i2c_not_started_msg);
		return ENC_SYM_EERROR;
	}

	uint8_t address = lbm_dec_as_u32(args[0]);
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(0, cmd, 50 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	xSemaphoreGive(i2c_mutex);

	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_gpio_configure(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	if (!lbm_is_number(args[0]) || !lbm_is_symbol(args[1])) {
		return ENC_SYM_EERROR;
	}

	int pin = lbm_dec_as_i32(args[0]);
	lbm_uint name = lbm_dec_sym(args[1]);

	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
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

static lbm_value ext_gpio_hold(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	int state = lbm_dec_as_i32(args[1]);

	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	if (state) {
		gpio_hold_en(pin);
	} else {
		gpio_hold_dis(pin);
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_gpio_hold_deepsleep(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int state = lbm_dec_as_i32(args[0]);

	if (state) {
		gpio_deep_sleep_hold_en();
	} else {
		gpio_deep_sleep_hold_dis();
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_gpio_write(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	int state = lbm_dec_as_i32(args[1]);

	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	gpio_set_level(pin, state);

	return ENC_SYM_TRUE;
}

static lbm_value ext_gpio_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int pin = lbm_dec_as_i32(args[0]);
	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(gpio_get_level(pin));
}

static lbm_value ext_main_init_done(lbm_value *args, lbm_uint argn) {
	(void)args;(void)argn;
	return main_init_done() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_crc16(lbm_value *args, lbm_uint argn) {
	if ((argn != 1 && argn != 2) || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
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

static lbm_value ext_crc32(lbm_value *args, lbm_uint argn) {
	if ((argn != 2 && argn != 3) || !lbm_is_array_r(args[0]) || !lbm_is_number(args[1])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	unsigned int len = array->size;
	if (argn == 3) {
		if (!lbm_is_number(args[2])) {
			return ENC_SYM_TERROR;
		}

		len = lbm_dec_as_u32(args[2]);
		if (len > array->size) {
			len = array->size;
		}
	}

	return lbm_enc_u32(crc32_with_init((uint8_t*)array->data, len, lbm_dec_as_u32(args[1])));
}

/**
 * signature: (buf-resize arr:array delta-size:number|nil [new-size:number] [copy-arr:copy-symbol])
 * -> array
 * where
 *   copy-symbol = 'copy|'mut
 *
 * If delta-size is passed, this extension calculates the new size by
 * adding the relative size to the current size, otherwise new-size is simply
 * used for the new size.
 *
 * If the new size is smaller than the current size, the array is just shrunk in
 * place without allocating a new buffer. Either delta-size or new-size must not
 * be nil.
 *
 * Either way, the passed array is resized mutably, with the returned reference
 * only for convenience, unless the symbol 'copy at the end. This makes it
 * always allocate a new buffer with the new size. You can also pass the symbol
 * 'mut to signify the default behaviour for completeness. The precise position
 * of the copy symbol is not important as long as it's the last argument.
 */
static lbm_value ext_buf_resize(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(2, 4);

	bool should_copy = false;
	if (argn > 2 && lbm_is_symbol(args[argn - 1])) {
		lbm_uint sym = lbm_dec_sym(args[argn - 1]);
		if (compare_symbol(sym, &syms_vesc.copy)) {
			should_copy = true;
		} else if (compare_symbol(sym, &syms_vesc.mut)) {
			should_copy = false;
		} else {
			lbm_set_error_suspect(args[argn - 1]);
			return ENC_SYM_TERROR;
		}
	}

	if ((!should_copy && !lbm_is_array_rw(args[0]))
		|| (should_copy && !lbm_is_array_r(args[0]))) {
		lbm_set_error_suspect(args[0]);
		return ENC_SYM_TERROR;
	}

	bool delta_size_passed = !lbm_is_symbol_nil(args[1]);
	bool new_size_passed   = argn > 2 && lbm_is_number(args[2]);

	if (delta_size_passed && !lbm_is_number(args[1])) {
		lbm_set_error_suspect(args[1]);
		return ENC_SYM_TERROR;
	}

	if (argn == 4 && !lbm_is_number(args[2])) {
		// The case where argn is 3 is covered by the first check.
		lbm_set_error_suspect(args[2]);
		return ENC_SYM_TERROR;
	}

	if (!delta_size_passed && !new_size_passed) {
		lbm_set_error_reason(
			"delta-size (arg 2) was nil while new-size wasn't provided (arg 3)"
		);
		return ENC_SYM_EERROR;
	}

	lbm_array_header_t *header = lbm_dec_array_header(args[0]);
	if (header == NULL) {
		// Should be impossible, unless it contained null pointer to header.
		return ENC_SYM_FATAL_ERROR;
	}

	uint32_t new_size;
	{
		int32_t new_size_signed;
		if (delta_size_passed) {
			new_size_signed = header->size + lbm_dec_as_i32(args[1]);
		} else {
			new_size_signed = lbm_dec_as_i32(args[2]);
		}

		if (new_size_signed < 0) {
			lbm_set_error_reason("resulting size was negative");
			return ENC_SYM_EERROR;
		}
		new_size = (uint32_t)new_size_signed;
	}

	if (should_copy) {
		void *buffer = lbm_malloc(new_size);
		if (!buffer) {
			return ENC_SYM_MERROR;
		}

		memcpy(buffer, header->data, MIN(header->size, new_size));
		if (new_size > header->size) {
			memset(buffer + header->size, 0, new_size - header->size);
		}

		lbm_value result;
		if (!lbm_lift_array(&result, buffer, new_size)) {
			return ENC_SYM_MERROR;
		}
		return result;
	} else {
		if (new_size == header->size) {
			return args[0];
		} else if (new_size < header->size) {
			uint32_t allocated_size = new_size;
			if (new_size == 0) {
				// arrays of size 0 still need some memory allocated for them.
				allocated_size = 1;
			}
			// We sadly can't trust the return value, as it fails if the allocation
			// was previously a single word long. So we just throw it away.
			lbm_memory_shrink_bytes(header->data, allocated_size);

			header->size = new_size;

			return args[0];
		} else {
			void *buffer = lbm_malloc(new_size);
			if (buffer == NULL) {
				return ENC_SYM_MERROR;
			}

			memcpy(buffer, header->data, header->size);
			memset(buffer + header->size, 0, new_size - header->size);

			lbm_memory_free(header->data);
			header->data = buffer;
			header->size = new_size;

			return args[0];
		}
	}
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
	uint8_t *buffer = mempools_get_lbm_packet_buffer();

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

typedef struct {
	lbm_cid id;
	uint16_t rate_ms;
	int num_uart;
	int pin_rx;
	int pin_tx;
} ublox_init_args;

static void ublox_init_task(void *arg) {
	int restart_cnt = lispif_get_restart_cnt();
	ublox_init_args *a = (ublox_init_args*)arg;

	bool res = ublox_init(false, a->rate_ms, a->num_uart, a->pin_rx, a->pin_tx);

	if (restart_cnt == lispif_get_restart_cnt()) {
		lbm_unblock_ctx_unboxed(a->id, res ? ENC_SYM_TRUE : ENC_SYM_NIL);
	}

	vTaskDelete(NULL);
}

static lbm_value ext_ublox_init(lbm_value *args, lbm_uint argn) {
	if (argn > 4) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	uint16_t rate = 500;
	if (argn >= 1) {
		rate = lbm_dec_as_i32(args[0]);
	}

	unsigned int uart_num = UART_NUM;
	if (argn >= 2) {
		uart_num = lbm_dec_as_i32(args[1]);
	}

	int pin_rx = UART_RX;
	if (argn >= 3) {
		pin_rx = lbm_dec_as_i32(args[2]);
	}

	int pin_tx = UART_TX;
	if (argn >= 4) {
		pin_tx = lbm_dec_as_i32(args[3]);
	}

	if (!utils_gpio_is_valid(pin_rx) || !utils_gpio_is_valid(pin_tx)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	if (uart_num >= UART_NUM_MAX) {
		lbm_set_error_reason("Invalid UART port");
		return ENC_SYM_EERROR;
	}

	static ublox_init_args a;
	a.id = lbm_get_current_cid();
	a.rate_ms = rate;
	a.num_uart = uart_num;
	a.pin_rx = pin_rx;
	a.pin_tx = pin_tx;

	xTaskCreatePinnedToCore(ublox_init_task, "Ublox Init", 2048, &a, 7, NULL, tskNO_AFFINITY);
	lbm_block_ctx_from_extension();

	return ENC_SYM_NIL;
}

static lbm_value ext_nmea_parse(lbm_value *args, lbm_uint argn) {
	if (argn != 1) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	char *str = lbm_dec_str(args[0]);
	if (!str) {
		return ENC_SYM_TERROR;
	}

	return nmea_decode_string(str) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_set_pos_time(lbm_value *args, lbm_uint argn) {
	nmea_state_t *s = nmea_get_state();

	int gga_cnt = s->gga_cnt;
	int rmc_cnt = s->rmc_cnt;

	for (lbm_uint i = 0;i < argn;i++) {
		lbm_value arg = args[i];

		if (lbm_is_number(arg)) {
			switch (i) {
			case 0:
			case 1:
			case 2:
			case 4:
				if (s->gga_cnt == gga_cnt) {
					s->gga_cnt++;
					s->gga.update_time = xTaskGetTickCount();
				}
				break;

			case 3:
			case 6:
			case 7:
			case 8:
				if (s->rmc_cnt == rmc_cnt) {
					s->rmc_cnt++;
					s->rmc.update_time = xTaskGetTickCount();
				}
				break;
			}

			switch (i) {
			case 0: {
				portDISABLE_INTERRUPTS();
				s->gga.lat = lbm_dec_as_double(arg);
				portENABLE_INTERRUPTS();
			} break;

			case 1: {
				portDISABLE_INTERRUPTS();
				s->gga.lon = lbm_dec_as_double(arg);
				portENABLE_INTERRUPTS();
			} break;

			case 2: {
				s->gga.height = lbm_dec_as_float(arg);
			} break;

			case 3: {
				s->rmc.speed = lbm_dec_as_float(arg);
			} break;

			case 4: {
				s->gga.h_dop = lbm_dec_as_float(arg);
			} break;

			case 5: {
				s->gga.ms_today = lbm_dec_as_u32(arg);

				s->rmc.ms = s->gga.ms_today % 1000;
				s->rmc.ss = (s->gga.ms_today / 1000) % 60;
				s->rmc.mm = (s->gga.ms_today / 1000 / 60) % 60;
				s->rmc.hh = (s->gga.ms_today / 1000 / 60 / 60) % 24;

				if (s->gga_cnt == gga_cnt) {
					s->gga_cnt++;
					s->gga.update_time = xTaskGetTickCount();
				}

				if (s->rmc_cnt == rmc_cnt) {
					s->rmc_cnt++;
					s->rmc.update_time = xTaskGetTickCount();
				}
			} break;

			case 6: {
				s->rmc.yy = lbm_dec_as_i32(arg);
			} break;

			case 7: {
				s->rmc.mo = lbm_dec_as_i32(arg);
			} break;

			case 8: {
				s->rmc.dd = lbm_dec_as_i32(arg);
			} break;

			default: break;
			}
		}
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_sleep_deep(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	esp_wifi_stop();

	float sleep_time = lbm_dec_as_float(args[0]);
	if (sleep_time > 0) {
		esp_sleep_enable_timer_wakeup((uint32_t)(sleep_time * 1.0e6));
	}

	esp_deep_sleep_start();

	return ENC_SYM_TRUE;
}

static lbm_value ext_sleep_light(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	esp_wifi_stop();

	float sleep_time = lbm_dec_as_float(args[0]);
	if (sleep_time > 0) {
		esp_sleep_enable_timer_wakeup((uint32_t)(sleep_time * 1.0e6));
	}

	esp_light_sleep_start();

	return ENC_SYM_TRUE;
}

static lbm_value ext_sleep_config_wakeup_pin(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int pin = lbm_dec_as_i32(args[0]);
	int mode = lbm_dec_as_i32(args[1]);

	if (!utils_gpio_is_valid(pin) || !esp_sleep_is_valid_wakeup_gpio(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	gpio_set_direction(pin, GPIO_MODE_INPUT);
	esp_deep_sleep_enable_gpio_wakeup(1 << pin,
			mode ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW);

	return ENC_SYM_TRUE;
}

RTC_DATA_ATTR char rtc_data[4096];

static lbm_value ext_rtc_data(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	lbm_value result;
	if (!lbm_lift_array(&result, rtc_data, (sizeof(rtc_data)))) {
		return ENC_SYM_MERROR;
	}

	return result;
}

static lbm_value ext_empty(lbm_value *args, lbm_uint argn) {
	(void)args;(void)argn;
	return ENC_SYM_TRUE;
}

// Remote Messages
#define RMSG_SLOT_NUM	8

typedef struct {
	lbm_cid cid;
	TickType_t start_time;
	float timeout_secs;
} rmsg_state;

static volatile bool event_task_running = false;
static SemaphoreHandle_t rmsg_mutex;
static volatile rmsg_state rmsg_slots[RMSG_SLOT_NUM];

static void event_task(void *arg) {
	for (;;) {
		xSemaphoreTake(rmsg_mutex, portMAX_DELAY);
		for (int i = 0;i < RMSG_SLOT_NUM;i++) {
			volatile rmsg_state *s = &rmsg_slots[i];
			if (s->cid >= 0 && s->timeout_secs > 0.0 && UTILS_AGE_S(s->start_time) > s->timeout_secs) {
				lbm_unblock_ctx_unboxed(s->cid, ENC_SYM_TIMEOUT);
				s->cid = -1;
			}
		}
		xSemaphoreGive(rmsg_mutex);

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

// Remote Messages

// (canmsg-recv slot timeout)
static lbm_value ext_canmsg_recv(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	int slot = lbm_dec_as_i32(args[0]);
	float timeout = lbm_dec_as_float(args[1]);

	if (slot < 0 || slot >= RMSG_SLOT_NUM) {
		return ENC_SYM_TERROR;
	}

	xSemaphoreTake(rmsg_mutex, portMAX_DELAY);
	rmsg_slots[slot].cid = lbm_get_current_cid();
	rmsg_slots[slot].start_time = xTaskGetTickCount();
	rmsg_slots[slot].timeout_secs = timeout;
	xSemaphoreGive(rmsg_mutex);

	lbm_block_ctx_from_extension();

	return ENC_SYM_TRUE;
}

// (canmsg-send can-id slot msg)
static lbm_value ext_canmsg_send(lbm_value *args, lbm_uint argn) {
	if (argn != 3 ||
			!lbm_is_number(args[0]) ||
			!lbm_is_number(args[1]) ||
			!lbm_is_array_r(args[2])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	int can_id = lbm_dec_as_i32(args[0]);
	int slot = lbm_dec_as_i32(args[1]);
	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[2]);

	if (can_id < 0 || can_id > 254) {
		return ENC_SYM_TERROR;
	}

	if (slot < 0 || slot >= RMSG_SLOT_NUM) {
		return ENC_SYM_TERROR;
	}

	if (array == NULL) {
		return ENC_SYM_TERROR;
	}

	if (array->size > 500) {
		return ENC_SYM_TERROR;
	}

	uint8_t *buf = mempools_get_lbm_packet_buffer();
	buf[0] = COMM_LISP_RMSG;
	buf[1] = slot;
	memcpy(buf + 2, array->data, array->size);
	comm_can_send_buffer(can_id, buf, array->size + 2, 2);
	mempools_free_packet_buffer(buf);

	return ENC_SYM_TRUE;
}

// File System
#define MAX_FILES 5
static FILE *files_open[MAX_FILES + 1] = {0};
static int file_now = 0;
static const char* str_f_not_open = "File not open.";

static char* dec_str_check(lbm_value val) {
	char *res = 0;
	if (lbm_is_array_r(val)) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(val);
		if (array) {
			res = (char *)array->data;

			if (res[array->size - 1] != '\0') {
				res = 0;
			}
		}
	}
	return res;
}

static FILE* file_from_arg(lbm_value arg) {
	uint32_t fn = lbm_dec_as_u32(arg);
	FILE *f = 0;
	for (int i = 0;i < MAX_FILES;i++) {
		if ((FILE*)fn == files_open[i]) {
			f = files_open[i];
			break;
		}
	}

	return f;
}

// (f-connect pin-mosi pin-miso pin-sck pin-cs optSpiSpeed) -> t or nil
static lbm_value ext_f_connect(lbm_value *args, lbm_uint argn) {
	if (argn != 4 && argn != 5) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	int pin_mosi = lbm_dec_as_u32(args[0]);
	int pin_miso = lbm_dec_as_u32(args[1]);
	int pin_sck = lbm_dec_as_u32(args[2]);
	int pin_cs = lbm_dec_as_u32(args[3]);

	int spi_speed = SDMMC_FREQ_DEFAULT;
	if (argn >= 5) {
		spi_speed = lbm_dec_as_u32(args[4]);
	}

	if (!utils_gpio_is_valid(pin_mosi) ||
			!utils_gpio_is_valid(pin_miso) ||
			!utils_gpio_is_valid(pin_sck) ||
			!utils_gpio_is_valid(pin_cs)) {
		lbm_set_error_reason((char*)string_pin_invalid);
		return ENC_SYM_TERROR;
	}

	esp_err_t res = log_mount_card(pin_mosi, pin_miso, pin_sck, pin_cs, spi_speed);

	if (res == ESP_OK) {
		return ENC_SYM_TRUE;
	} else {
		lbm_set_esp_error_reason(res);
		return ENC_SYM_EERROR;
	}
}

// (f-connect-nand pin-mosi pin-miso pin-sck pin-cs optSpiSpeed) -> t or nil
static lbm_value ext_f_connect_nand(lbm_value *args, lbm_uint argn) {
	if (argn != 4 && argn != 5) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	int pin_mosi = lbm_dec_as_u32(args[0]);
	int pin_miso = lbm_dec_as_u32(args[1]);
	int pin_sck = lbm_dec_as_u32(args[2]);
	int pin_cs = lbm_dec_as_u32(args[3]);

	int spi_speed = SDMMC_FREQ_DEFAULT;
	if (argn >= 5) {
		spi_speed = lbm_dec_as_u32(args[4]);
	}

	if (!utils_gpio_is_valid(pin_mosi) ||
			!utils_gpio_is_valid(pin_miso) ||
			!utils_gpio_is_valid(pin_sck) ||
			!utils_gpio_is_valid(pin_cs)) {
		lbm_set_error_reason((char*)string_pin_invalid);
		return ENC_SYM_TERROR;
	}

	bool res = log_mount_nand_flash(pin_mosi, pin_miso, pin_sck, pin_cs, spi_speed);

	return res ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (f-disconnect) -> t
static lbm_value ext_f_disconnect(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	log_unmount_card();
	log_unmount_nand_flash();
	return ENC_SYM_TRUE;
}

// (f-open path mode) -> file
static lbm_value ext_f_open(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	if (file_now >= MAX_FILES) {
		lbm_set_error_reason("Too many files open.");
		return ENC_SYM_EERROR;
	}

	char *path = dec_str_check(args[0]);
	char *mode = dec_str_check(args[1]);

	if (!path || !mode) {
		return ENC_SYM_TERROR;
	}

	char path_full[strlen(path) + strlen(file_basepath) + 1];
	strcpy(path_full, file_basepath);
	strcat(path_full, path);

	FILE *f = fopen(path_full, mode);

	if (f) {
		files_open[file_now++] = f;
		return lbm_enc_u32((uint32_t)f);
	} else {
		return ENC_SYM_NIL;
	}
}

// (f-close file) -> t, nil
static lbm_value ext_f_close(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t fn = lbm_dec_as_u32(args[0]);
	FILE *f = 0;
	int f_ind = 0;
	for (int i = 0;i < MAX_FILES;i++) {
		if ((FILE*)fn == files_open[i]) {
			f = files_open[i];
			f_ind = i;
			break;
		}
	}

	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	for (int i = f_ind;i < file_now;i++) {
		files_open[i] = files_open[i + 1];
	}

	file_now--;

	fclose(f);

	return ENC_SYM_TRUE;
}

// (f-read file size) -> array
static lbm_value ext_f_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	int32_t sz = lbm_dec_as_i32(args[1]);

	lbm_value res = ENC_SYM_MERROR;
	if (lbm_create_array(&res, sz)) {
		lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);
		int rd = fread(arr->data, 1, sz, f);
		if (rd == 0) {
			lbm_free(arr->data);
			arr->data = 0;
			res = ENC_SYM_NIL;
		} else if (rd < 0) {
			lbm_free(arr->data);
			arr->data = 0;
			res = ENC_SYM_EERROR;
		} else {
			if (rd < sz) {
				lbm_memory_shrink_bytes(arr->data, rd);
				arr->size = rd;
			}
		}
	}

	return res;
}

// (f-readline file maxlen) -> array
static lbm_value ext_f_readline(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	int32_t sz = lbm_dec_as_i32(args[1]);

	lbm_value res = ENC_SYM_MERROR;
	if (lbm_create_array(&res, sz)) {
		lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);

		char *rd = fgets((char*)arr->data, sz, f);
		if (rd == 0) {
			lbm_free(arr->data);
			arr->data = 0;
			res = ENC_SYM_NIL;
		} else {
			int len_rd = strnlen(rd, sz);
			if ((len_rd + 1) < sz) {
				lbm_memory_shrink(arr->data, len_rd + 1);
				arr->size = len_rd + 1;
			}
		}
	}

	return res;
}

// (f-write file buf) -> t, nil
static lbm_value ext_f_write(lbm_value *args, lbm_uint argn) {
	if (!lbm_is_number(args[0]) || !lbm_is_array_r(args[1])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
	lbm_value res = ENC_SYM_EERROR;
	if (array) {
		int wr = fwrite(array->data, 1, array->size, f);
		if (wr == array->size) {
			res = ENC_SYM_TRUE;
		} else {
			lbm_set_error_reason("Could not write to file.");
		}
	}

	return res;
}

// (f-tell file) -> position
static lbm_value ext_f_tell(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i32(ftell(f));
}

// (f-seek file pos) -> t, nil
static lbm_value ext_f_seek(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	int32_t pos = lbm_dec_as_i32(args[1]);
	int32_t pos_now = ftell(f);
	if ((pos_now + pos) < 0) {
		lbm_set_error_reason("Cannot seek past the beginning of file.");
		return ENC_SYM_EERROR;
	}

	return fseek(f, pos, pos >= 0 ? SEEK_SET : SEEK_END) == 0 ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (f-mkdir path) -> t, nil
static lbm_value ext_f_mkdir(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	char *path = dec_str_check(args[0]);
	if (!path) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	char path_full[strlen(path) + strlen(file_basepath) + 1];
	strcpy(path_full, file_basepath);
	strcat(path_full, path);

	return mkdir(path_full, 0775) == 0 ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (f-rm path) -> t, nil
static lbm_value ext_f_rm(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	char *path = dec_str_check(args[0]);
	if (!path) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	char path_full[strlen(path) + strlen(file_basepath) + 1];
	strcpy(path_full, file_basepath);
	strcat(path_full, path);

	return utils_rmtree(path_full) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (f-ls path) -> ((path is-dir) (path is-dir) ...)
// (f-ls path 'size) -> ((path is-dir size) (path is-dir size) ...)
static lbm_value ext_f_ls(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_RANGE(1, 2);

	char *path = dec_str_check(args[0]);
	if (!path) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	bool report_size = false;
	if (argn  == 2) {
		if (lbm_is_symbol(args[1]) && lbm_dec_sym(args[1]) == sym_size) {
			report_size = true;
		}
	}

	char path_full[strlen(path) + strlen(file_basepath) + 1];
	strcpy(path_full, file_basepath);
	strcat(path_full, path);

	lbm_value res = ENC_SYM_NIL;

	bool merror = false;
	struct dirent *dir;
	DIR *d = opendir(path_full);
	if (d) {
		while ((dir = readdir(d)) != NULL) {
			lbm_value current = ENC_SYM_NIL;

			int len_f = strlen(dir->d_name);

			if (report_size){
				char path_file[strlen(path_full) + strlen(dir->d_name) + 2];
				strcpy(path_file, path_full);
				strcat(path_file, "/");
				strcat(path_file, dir->d_name);

				size_t size = 0;
				if (dir->d_type != DT_DIR) {
					FILE *f = fopen(path_file, "r");
					if (f) {
						fseek(f, 0, SEEK_END);
						size = ftell(f);
						fclose(f);
					}
				} else {
					DIR *d2 = opendir(path_file);
					if (d2) {
						struct dirent *dir2;
						while ((dir2 = readdir(d2)) != NULL) {
							size++;
						}
						closedir(d2);
					}
				}
				current = lbm_cons(lbm_enc_i(size), current);
			}

			current = lbm_cons(dir->d_type == DT_DIR ? ENC_SYM_TRUE : ENC_SYM_NIL, current);

			lbm_value name_buf = ENC_SYM_MERROR;
			if (lbm_create_array(&name_buf, len_f + 1)) {
				lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(name_buf);
				strcpy((char*)arr->data, dir->d_name);
				current = lbm_cons(name_buf, current);
			} else {
				merror = true;
				break;
			}

			res = lbm_cons(current, res);
		}

		closedir(d);
	}

	if (merror) {
		return ENC_SYM_MERROR;
	} else {
		return res;
	}
}

// (f-size path/file) -> size
static lbm_value ext_f_size(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	int32_t sz = -1;

	if (lbm_is_number(args[0])) {
		FILE *f = file_from_arg(args[0]);
		if (!f) {
			lbm_set_error_reason((char*)str_f_not_open);
			return ENC_SYM_EERROR;
		}

		uint32_t pos_old = ftell(f);
		fseek(f, 0, SEEK_END);
		sz = ftell(f);
		fseek(f, pos_old, SEEK_SET);
	} else {
		char *path = dec_str_check(args[0]);
		if (!path) {
			lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
			return ENC_SYM_TERROR;
		}

		char path_full[strlen(path) + strlen(file_basepath) + 1];
		strcpy(path_full, file_basepath);
		strcat(path_full, path);

		FILE *f = fopen(path_full, "r");
		if (f) {
			fseek(f, 0, SEEK_END);
			sz = ftell(f);
			fclose(f);
		}
	}

	return lbm_enc_i32(sz);
}

// (f-rename oldname newname)
static lbm_value ext_f_rename(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	char *old_name = dec_str_check(args[0]);
	if (!old_name) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	char *new_name = dec_str_check(args[1]);
	if (!new_name) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	char *old_full = lbm_malloc(strlen(old_name) + strlen(file_basepath) + 1);
	if (!old_full) {
		return ENC_SYM_MERROR;
	}

	char *new_full = lbm_malloc(strlen(new_name) + strlen(file_basepath) + 1);
	if (!new_full) {
		lbm_free(old_full);
		return ENC_SYM_MERROR;
	}

	strcpy(old_full, file_basepath);
	strcat(old_full, old_name);

	strcpy(new_full, file_basepath);
	strcat(new_full, new_name);

	lbm_value res = rename(old_full, new_full) == 0 ? ENC_SYM_TRUE : ENC_SYM_NIL;

	lbm_free(old_full);
	lbm_free(new_full);

	return res;
}

// (f-sync file) -> t, nil
static lbm_value ext_f_sync(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	FILE *f = file_from_arg(args[0]);
	if (!f) {
		lbm_set_error_reason((char*)str_f_not_open);
		return ENC_SYM_EERROR;
	}

	return fsync(fileno(f)) == 0 ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (f-fatinfo) -> (MB-free MB-total)
static lbm_value ext_f_fatinfo(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	// Remove last / in name
	char basepath[strlen(file_basepath)];
	strcpy(basepath, file_basepath);
	basepath[strlen(file_basepath) - 1] = '\0';

	uint64_t total = 0;
	uint64_t free = 0;
	esp_vfs_fat_info(basepath, &total, &free);
	total /= 1024;
	total /= 1024;
	free /= 1024;
	free /= 1024;

	lbm_value res = ENC_SYM_NIL;
	res = lbm_cons(lbm_enc_i(total), res);
	res = lbm_cons(lbm_enc_i(free), res);

	return res;
}

typedef struct {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t test_version;
	char commit_hash[47];
	char user_commit_hash[47];
} fw_info_t;

static send_func_t fw_send_func_old;
static volatile bool fw_reply_rx = false;
static volatile bool fw_reply_ok = false;
static volatile fw_info_t fw_reply_fw_info = {0};
static volatile lbm_cid fw_rx_cid = -1;

static void fw_reply_func(unsigned char *data, unsigned int len) {
	if (len < 2) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_ERASE_NEW_APP:
	case COMM_WRITE_NEW_APP_DATA:
	case COMM_LISP_ERASE_CODE:
	case COMM_QMLUI_ERASE:
	case COMM_LISP_WRITE_CODE:
	case COMM_QMLUI_WRITE:
	case COMM_LISP_SET_RUNNING:
		fw_reply_ok = data[0];

		if (fw_rx_cid >= 0) {
			lbm_unblock_ctx_unboxed(fw_rx_cid, fw_reply_ok ? ENC_SYM_TRUE : ENC_SYM_NIL);
		}
		break;
	case COMM_FW_INFO: {
		uint32_t index                 = 0;
		fw_reply_fw_info.version_major = data[index++];
		fw_reply_fw_info.version_minor = data[index++];
		fw_reply_fw_info.test_version  = data[index++];

		strncpy(
			(char *)fw_reply_fw_info.commit_hash, (char *)&data[index],
			sizeof(fw_reply_fw_info.commit_hash) - 1
		);
		size_t commit_hash_size = strlen((char *)fw_reply_fw_info.commit_hash) + 1;
		index += commit_hash_size;

		strncpy(
			(char *)fw_reply_fw_info.user_commit_hash, (char *)&data[index],
			sizeof(fw_reply_fw_info.user_commit_hash) - 1
		);
		size_t user_commit_hash_size =
			strlen((char *)fw_reply_fw_info.user_commit_hash) + 1;
		index += user_commit_hash_size;

		if (fw_rx_cid >= 0) {
			lbm_flat_value_t v;

			// If start flatten succeeds the f_*-operations will also succeed if the
			// size calculation here is correct.
			if (!start_flatten_with_gc(&v, 65 + commit_hash_size + user_commit_hash_size)) {
				lbm_unblock_ctx_unboxed(fw_rx_cid, ENC_SYM_MERROR);
			}

			f_cons(&v); // +1

			// ('version . (uint uint))
			f_cons(&v); // +1
			// Ensure that the symbol is added.
			compare_symbol(0, &syms_vesc.version);
			f_sym(&v, syms_vesc.version); // +5

			f_cons(&v); // +1
			f_u(&v, (lbm_uint)fw_reply_fw_info.version_major); // +5

			f_cons(&v); // +1
			f_u(&v, (lbm_uint)fw_reply_fw_info.version_minor); // +5
			f_sym(&v, SYM_NIL);                    // +5

			f_cons(&v); // +1

			// ('test_version . uint)
			f_cons(&v); // + 1
			compare_symbol(0, &syms_vesc.test_version);
			f_sym(&v, syms_vesc.test_version); // +5
			f_u(&v, (lbm_uint)fw_reply_fw_info.test_version); // +5

			f_cons(&v); // +1

			// ('commit . str)
			f_cons(&v); // +1
			compare_symbol(0, &syms_vesc.commit);
			f_sym(&v, syms_vesc.commit); // +5
			if (fw_reply_fw_info.commit_hash[0] == '\0') {
				f_sym(&v, SYM_NIL);
			} else {
				f_lbm_array(&v, commit_hash_size,
						(uint8_t*) fw_reply_fw_info.commit_hash);
			} // +5 + user_commit_hash_size
			f_cons(&v); // +1

			// ('user-commit . str)
			f_cons(&v); // +1
			compare_symbol(0, &syms_vesc.user_commit);
			f_sym(&v, syms_vesc.user_commit); // +5
			if (fw_reply_fw_info.user_commit_hash[0] == '\0') {
				f_sym(&v, SYM_NIL);
			} else {
				f_lbm_array(&v, user_commit_hash_size,
						(uint8_t*) fw_reply_fw_info.user_commit_hash);
			} // +5 + user_commit_hash_size
			f_sym(&v, SYM_NIL); // +5

			lbm_finish_flatten(&v);

			if (!lbm_unblock_ctx(fw_rx_cid, &v)) {
				lbm_free(v.buf);
				lbm_unblock_ctx_unboxed(fw_rx_cid, ENC_SYM_EERROR);
			}
		}
	} break;

	case COMM_BMS_BLNC_SELFTEST: {
		lbm_flat_value_t v;

		if (fw_rx_cid >= 0) {
			int32_t ind = 0;
			int ok_ic = buffer_get_int16(data, &ind);
			int cell_num = (len - 2) / 8;

			bool ok_all = true;
			for (int i = 0; i < cell_num;i++) {
				float no_bal = buffer_get_float32_auto(data, &ind);
				float bal = buffer_get_float32_auto(data, &ind);
				float diff = fabsf(bal - no_bal);
				bool ok = (diff / no_bal > 0.03 && bal > 2.0);
				if (!ok) {
					ok_all = false;
					break;
				}
			}

			bool result = start_flatten_with_gc(&v, 25 + 24 * cell_num);
			if (result) {
				ind = 2;

				f_cons(&v); // +1
				f_i(&v, ok_ic); // +5
				f_cons(&v); // +1
				f_i(&v, ok_all); // +5
				f_cons(&v); // +1
				f_i(&v, cell_num); // +5

				for (int i = 0; i < cell_num;i++) { // + 24 * cells
					f_cons(&v); // +1

					float no_bal = buffer_get_float32_auto(data, &ind);
					float bal = buffer_get_float32_auto(data, &ind);
					float diff = fabsf(bal - no_bal);
					bool ok = (diff / no_bal > 0.03 && bal > 2.0);

					f_cons(&v); // +1
					f_i(&v, ok); // +5
					f_cons(&v); // +1
					f_float(&v, no_bal); // +5
					f_cons(&v); // +1
					f_float(&v, bal); // +5
					f_sym(&v, ENC_SYM_NIL); // +5
				}

				f_sym(&v, ENC_SYM_NIL); // +5
				lbm_finish_flatten(&v);

				if (!lbm_unblock_ctx(fw_rx_cid, &v)) {
					lbm_free(v.buf);
					lbm_unblock_ctx_unboxed(fw_rx_cid, ENC_SYM_EERROR);
				}
			} else {
				lbm_unblock_ctx_unboxed(fw_rx_cid, ENC_SYM_MERROR);
			}
		}
	} break;

	default:
		return;
	}

	fw_reply_rx = true;
	fw_rx_cid = -1;

	commands_set_send_func(fw_send_func_old);
}

static lbm_value fw_lbm_qml_write(lbm_value *args, lbm_uint argn, COMM_PACKET_ID cmd) {
	if (argn > 3 || !lbm_is_number(args[0]) ||
			!lbm_is_array_r(args[1]) || (argn == 3 && !lbm_is_number(args[2]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 3) {
		can_id = lbm_dec_as_i32(args[2]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint32_t offset = lbm_dec_as_u32(args[0]);

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);
	lbm_value res = ENC_SYM_EERROR;
	if (array) {
		if (array->size > 500) {
			lbm_set_error_reason("At most 500 bytes can be written at a time.");
			return ENC_SYM_TERROR;
		}

		uint8_t *buf = mempools_get_lbm_packet_buffer();
		int32_t ind = 0;

		if (can_id >= 0) {
			buf[ind++] = COMM_FORWARD_CAN;
			buf[ind++] = can_id;
		}

		buf[ind++] = cmd;
		buffer_append_uint32(buf, offset, &ind);
		memcpy(buf + ind, array->data, array->size);
		ind += array->size;

		fw_reply_rx = false;
		fw_reply_ok = false;
		fw_rx_cid = -1;
		fw_send_func_old = commands_get_send_func();
		commands_process_packet(buf, ind, fw_reply_func);
		fw_rx_cid = lbm_get_current_cid();
		mempools_free_packet_buffer(buf);

		if (fw_reply_rx) {
			res = fw_reply_ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
		} else {
			lbm_block_ctx_from_extension_timeout(1.5);
			res = ENC_SYM_TRUE;
		}
	}

	return res;
}

static lbm_value lbm_qml_erase(lbm_value *args, lbm_uint argn, COMM_PACKET_ID cmd) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 1) {
		can_id = lbm_dec_as_i32(args[0]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[8];
	int32_t ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
	}

	buf[ind++] = cmd;
	if (can_id < 0 && cmd == COMM_LISP_ERASE_CODE) {
		buffer_append_int32(buf, -2, &ind);
	}

	fw_reply_rx = false;
	fw_reply_ok = false;
	fw_rx_cid = -1;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, fw_reply_func);
	fw_rx_cid = lbm_get_current_cid();

	if (fw_reply_rx) {
		return fw_reply_ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
	} else {
		lbm_block_ctx_from_extension_timeout(20.0);
		return ENC_SYM_TRUE;
	}
}

// (fw-erase size optCanId) -> t, nil
static lbm_value ext_fw_erase(lbm_value *args, lbm_uint argn) {
	if (argn > 2 || !lbm_is_number(args[0]) || (argn == 2 && !lbm_is_number(args[1]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 2) {
		can_id = lbm_dec_as_i32(args[1]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[8];
	int32_t ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
	}

	buf[ind++] = COMM_ERASE_NEW_APP;
	buffer_append_uint32(buf, lbm_dec_as_i32(args[0]), &ind);
	fw_reply_rx = false;
	fw_reply_ok = false;
	fw_rx_cid = -1;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, fw_reply_func);
	fw_rx_cid = lbm_get_current_cid();

	if (fw_reply_rx) {
		return fw_reply_ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
	} else {
		lbm_block_ctx_from_extension_timeout(20.0);
		return ENC_SYM_TRUE;
	}
}

// (fw-write offset data optCanId) -> t, nil
static lbm_value ext_fw_write(lbm_value *args, lbm_uint argn) {
	return fw_lbm_qml_write(args, argn, COMM_WRITE_NEW_APP_DATA);
}

// (fw-reboot optCanId) -> t, nil
static lbm_value ext_fw_reboot(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 1) {
		can_id = lbm_dec_as_i32(args[0]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[4];
	unsigned int ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
	}

	buf[ind++] = COMM_JUMP_TO_BOOTLOADER;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, 0);
	commands_set_send_func(fw_send_func_old);
	return ENC_SYM_TRUE;
}

// LBM and QML erase, upload, run

// (lbm-erase optCanId) -> t, nil
static lbm_value ext_lbm_erase(lbm_value *args, lbm_uint argn) {
	return lbm_qml_erase(args, argn, COMM_LISP_ERASE_CODE);
}

// (qml-erase optCanId) -> t, nil
static lbm_value ext_qml_erase(lbm_value *args, lbm_uint argn) {
	return lbm_qml_erase(args, argn, COMM_QMLUI_ERASE);
}

// (lbm-write offset data optCanId) -> t, nil
static lbm_value ext_lbm_write(lbm_value *args, lbm_uint argn) {
	return fw_lbm_qml_write(args, argn, COMM_LISP_WRITE_CODE);
}

// (qml-write offset data optCanId) -> t, nil
static lbm_value ext_qml_write(lbm_value *args, lbm_uint argn) {
	return fw_lbm_qml_write(args, argn, COMM_QMLUI_WRITE);
}

static const esp_partition_t *update_partition = NULL;
static const void *update_partition_data = NULL;
static esp_partition_mmap_handle_t update_partition_handle = 0;

static bool fw_map_buffer(void) {
	if (update_partition) {
		return true;
	}

	if (!update_partition) {
		update_partition = esp_ota_get_next_update_partition(NULL);

		esp_err_t res = esp_partition_mmap(update_partition, 0, update_partition->size,
				ESP_PARTITION_MMAP_DATA, &update_partition_data, &update_partition_handle);

		if (res != ESP_OK) {
			update_partition = NULL;
			lbm_set_error_reason("FW buffer mmap failed");
			return false;
		}
	}

	return true;
}

// (fw-data optOffset, optLen)
static lbm_value ext_fw_data(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	if (!fw_map_buffer()) {
		return ENC_SYM_EERROR;
	}

	uint32_t offset = 0;
	if (argn >= 1) {
		offset = lbm_dec_as_u32(args[0]);
	}

	uint32_t len = update_partition->size - offset;
	if (argn >= 2) {
		len = lbm_dec_as_u32(args[1]);
	}

	if (offset >= update_partition->size) {
		return ENC_SYM_EERROR;
	}

	if ((offset + len) > update_partition->size) {
		return ENC_SYM_EERROR;
	}

	lbm_value val;
	lbm_lift_array(&val, (char*)update_partition_data + offset, len);
	return val;
}

// (fw-write-raw offset data)
static lbm_value ext_fw_write_raw(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	if (!lbm_is_number(args[0])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	uint32_t offset = lbm_dec_as_u32(args[0]);

	lbm_array_header_t *arr_in = NULL;
	if (lbm_is_array_r(args[1])) {
		arr_in = (lbm_array_header_t *)lbm_car(args[1]);
	} else {
		return ENC_SYM_TERROR;
	}

	if (!fw_map_buffer()) {
		return ENC_SYM_EERROR;
	}

	if ((offset + arr_in->size) > update_partition->size) {
		lbm_set_error_reason("Trying to write outside of buffer");
		return ENC_SYM_EERROR;
	}

	esp_err_t res = esp_partition_write(update_partition, offset, arr_in->data, arr_in->size);

	return res == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// (fw-info optCanId) -> t|nil
static lbm_value ext_fw_info(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_TERROR;
	}

	/*
	Allocate potentially used return values up front.
	Return value shape:
	(
		('version . (uint uint))
		('test-version uint)
		('commit . str)
		('user-commit . str)
	)
	*/
	lbm_value assoc_list = lbm_allocate_empty_list(4);
	if (assoc_list == ENC_SYM_MERROR) {
		return ENC_SYM_MERROR;
	}
	lbm_value version_list = lbm_allocate_empty_list(2);
	if (version_list == ENC_SYM_MERROR) {
		return ENC_SYM_MERROR;
	}
	lbm_value property_cons_cells[4];
	for (size_t i = 0; i < 4; i++) {
		property_cons_cells[i] = lbm_cons(ENC_SYM_NIL, ENC_SYM_NIL);
		if (property_cons_cells[i] == ENC_SYM_MERROR) {
			return ENC_SYM_MERROR;
		}
	}
	lbm_value commit_str;
	if (!lbm_heap_allocate_array(&commit_str, 47)) {
		return ENC_SYM_MERROR;
	}
	lbm_value user_commit_str;
	if (!lbm_heap_allocate_array(&user_commit_str, 47)) {
		lbm_heap_explicit_free_array(user_commit_str);
		return ENC_SYM_MERROR;
	}

	int can_id = -1;
	if (argn == 1) {
		can_id = lbm_dec_as_i32(args[0]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[3];
	int32_t ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
	}

	buf[ind++] = COMM_FW_INFO;
	fw_reply_rx = false;
	fw_rx_cid = -1;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, fw_reply_func);
	fw_rx_cid = lbm_get_current_cid();

	if (fw_reply_rx) {
		// Ensure symbols have been added.
		compare_symbol(0, &syms_vesc.version);
		compare_symbol(0, &syms_vesc.test_version);
		compare_symbol(0, &syms_vesc.commit);
		compare_symbol(0, &syms_vesc.user_commit);

		size_t i = 0;
		for (lbm_value current = assoc_list; lbm_is_cons(current); current = lbm_cdr(current)) {
			lbm_set_car(current, property_cons_cells[i++]);
		}

		lbm_set_car_and_cdr(property_cons_cells[0], lbm_enc_sym(syms_vesc.version), version_list);
		{
			lbm_value current = version_list;
			lbm_set_car(current, lbm_enc_u(fw_reply_fw_info.version_major));
			current = lbm_cdr(current);
			lbm_set_car(current, lbm_enc_u(fw_reply_fw_info.version_minor));
		}
		lbm_set_car_and_cdr(property_cons_cells[1], lbm_enc_sym(syms_vesc.test_version), lbm_enc_u(fw_reply_fw_info.test_version));
		// TODO: Return nil for the strings that are empty.
		lbm_set_car_and_cdr(property_cons_cells[2], lbm_enc_sym(syms_vesc.commit), commit_str);
		{
			size_t len = strlen((char *)fw_reply_fw_info.commit_hash);
			if (len == 0) {
				lbm_set_cdr(property_cons_cells[3], ENC_SYM_NIL);
				lbm_heap_explicit_free_array(commit_str);
			} else {
				char *data = lbm_dec_str(commit_str);
				memcpy(data, (char *)fw_reply_fw_info.commit_hash, len + 1);
				lbm_array_shrink(commit_str, len + 1);
			}
		}
		lbm_set_car_and_cdr(property_cons_cells[3], lbm_enc_sym(syms_vesc.user_commit), user_commit_str);
		{
			size_t len = strlen((char *)fw_reply_fw_info.user_commit_hash);
			if (len == 0) {
				lbm_set_cdr(property_cons_cells[3], ENC_SYM_NIL);
				lbm_heap_explicit_free_array(user_commit_str);
			} else {
				char *data = lbm_dec_str(user_commit_str);
				memcpy(data, (char *)fw_reply_fw_info.user_commit_hash, len + 1);
				lbm_array_shrink(user_commit_str, len + 1);
			}
		}

		return assoc_list;
	} else {
		// Since the return value will be done via a flat value rather than an
		// lbm value, we will not be using these. The cons cells are freed
		// automatically.
		lbm_heap_explicit_free_array(commit_str);
		lbm_heap_explicit_free_array(user_commit_str);

		lbm_block_ctx_from_extension_timeout(2.0);
		return ENC_SYM_NIL;
	}
}

static void lbm_run_task(void *arg) {
	uint8_t buf[8];
	int32_t ind = 0;

	buf[ind++] = COMM_LISP_SET_RUNNING;
	buf[ind++] = (int32_t)arg;
	fw_reply_rx = false;
	fw_reply_ok = false;
	fw_rx_cid = -1;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, fw_reply_func);

	vTaskDelete(NULL);
}

// (lbm-run running optCanId) -> t, nil
static lbm_value ext_lbm_run(lbm_value *args, lbm_uint argn) {
	if (argn > 2 || !lbm_is_number(args[0]) || (argn == 2 && !lbm_is_number(args[1]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 2) {
		can_id = lbm_dec_as_i32(args[1]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[8];
	int32_t ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
		buf[ind++] = COMM_LISP_SET_RUNNING;
		buf[ind++] = lbm_dec_as_i32(args[0]);
		fw_reply_rx = false;
		fw_reply_ok = false;
		fw_rx_cid = -1;
		fw_send_func_old = commands_get_send_func();
		commands_process_packet(buf, ind, fw_reply_func);
		fw_rx_cid = lbm_get_current_cid();

		if (fw_reply_rx) {
			return fw_reply_ok ? ENC_SYM_TRUE : ENC_SYM_NIL;
		} else {
			lbm_block_ctx_from_extension_timeout(20.0);
			return ENC_SYM_TRUE;
		}
	} else {
		xTaskCreatePinnedToCore(lbm_run_task, "LBM Restart", 2048, (void*)lbm_dec_as_i32(args[0]), 7, NULL, tskNO_AFFINITY);
		return ENC_SYM_TRUE;
	}
}

// (bms-st optCanId) -> (ok_ic ok_all cell_num (ok no-bal bal) ...)
static lbm_value ext_bms_st(lbm_value *args, lbm_uint argn) {
	if (argn > 1 || (argn == 1 && !lbm_is_number(args[0]))) {
		return ENC_SYM_TERROR;
	}

	int can_id = -1;
	if (argn == 1) {
		can_id = lbm_dec_as_i32(args[0]);
		if (can_id > 254) {
			return ENC_SYM_TERROR;
		}
	}

	uint8_t buf[8];
	int32_t ind = 0;

	if (can_id >= 0) {
		buf[ind++] = COMM_FORWARD_CAN;
		buf[ind++] = can_id;
	}

	buf[ind++] = COMM_BMS_BLNC_SELFTEST;
	fw_reply_rx = false;
	fw_send_func_old = commands_get_send_func();
	commands_process_packet(buf, ind, fw_reply_func);
	fw_rx_cid = lbm_get_current_cid();

	if (fw_reply_rx) {
		return ENC_SYM_NIL;
	} else {
		lbm_block_ctx_from_extension_timeout(10.0);
		return ENC_SYM_NIL;
	}
}

// AS504x encoder

static AS504x_config_t as504x = {0};
static bool as504x_init_done = false;
static char *str_as504x_not_init_msg = "AS504x not initialized.";

static lbm_value ext_as504x_init(lbm_value *args, lbm_uint argn) {
	if (argn != 3 && argn != 4) {
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	int miso = lbm_dec_as_i32(args[0]);
	int sck = lbm_dec_as_i32(args[1]);
	int nss = lbm_dec_as_i32(args[2]);

	if (!utils_gpio_is_valid(miso) || !utils_gpio_is_valid(sck) || !utils_gpio_is_valid(nss)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	int mosi = -1;
	if (argn == 4) {
		mosi = lbm_dec_as_i32(args[3]);
		if (!utils_gpio_is_valid(mosi)) {
			lbm_set_error_reason(string_pin_invalid);
			return ENC_SYM_EERROR;
		}
	}

	as504x.sw_spi.miso_pin = miso;
	as504x.sw_spi.mosi_pin = mosi;
	as504x.sw_spi.sck_pin = sck;
	as504x.sw_spi.nss_pin = nss;

	as504x_init_done = true;

	return enc_as504x_init(&as504x) ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_as504x_deinit(lbm_value *args, lbm_uint argn) {
	if (!as504x_init_done) {
		lbm_set_error_reason(str_as504x_not_init_msg);
		return ENC_SYM_EERROR;
	}

	enc_as504x_deinit(&as504x);
	as504x_init_done = false;

	return ENC_SYM_TRUE;
}

static lbm_value ext_as504x_angle(lbm_value *args, lbm_uint argn) {
	if (!as504x_init_done) {
		lbm_set_error_reason(str_as504x_not_init_msg);
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float(enc_as504x_read_angle(&as504x));
}

// IMU

static imu_config imu_cfg;

static lbm_value ext_imu_start_lsm6(lbm_value *args, lbm_uint argn) {
	lbm_value res = ext_i2c_start(args, argn);

	if (res != ENC_SYM_TRUE) {
		return res;
	}

	memset(&imu_cfg, 0, sizeof(imu_cfg));
	imu_cfg.type = IMU_TYPE_EXTERNAL_LSM6DS3;
	imu_cfg.sample_rate_hz = 1000;
	imu_cfg.use_magnetometer = true;
	imu_cfg.filter = IMU_FILTER_MEDIUM;
	imu_cfg.accel_confidence_decay = 1.0;
	imu_cfg.mahony_kp = 0.3;
	imu_cfg.mahony_ki = 0.0;
	imu_cfg.madgwick_beta = 0.1;

	imu_init(&imu_cfg, i2c_mutex);

	return ENC_SYM_TRUE;
}

static lbm_value ext_imu_stop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	imu_stop();
	return ENC_SYM_TRUE;
}

static lbm_value ext_get_imu_rpy(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float rpy[3];
	imu_get_rpy(rpy);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(rpy[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(rpy[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(rpy[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_quat(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float q[4];
	imu_get_quaternions(q);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(q[3]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(q[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(q[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(q[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_acc(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float acc[3];
	imu_get_accel(acc);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(acc[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(acc[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(acc[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_gyro(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float gyro[3];
	imu_get_gyro(gyro);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(gyro[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(gyro[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(gyro[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_mag(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float mag[3];
	imu_get_mag(mag);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(mag[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(mag[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(mag[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_acc_derot(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float acc[3];
	imu_get_accel_derotated(acc);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(acc[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(acc[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(acc[0]), imu_data);

	return imu_data;
}

static lbm_value ext_get_imu_gyro_derot(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	float gyro[3];
	imu_get_gyro_derotated(gyro);

	lbm_value imu_data = ENC_SYM_NIL;
	imu_data = lbm_cons(lbm_enc_float(gyro[2]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(gyro[1]), imu_data);
	imu_data = lbm_cons(lbm_enc_float(gyro[0]), imu_data);

	return imu_data;
}

// UART
static SemaphoreHandle_t m_uart_mutex;
static bool uart_mutex_init_done = false;
static volatile int m_uart_number = -1;

// (uart-start uart-num rx-pin tx-pin baud)
static lbm_value ext_uart_start(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	int uart_num = lbm_dec_as_i32(args[0]);
	int rx_pin = lbm_dec_as_i32(args[1]);
	int tx_pin = lbm_dec_as_i32(args[2]);
	int baud = lbm_dec_as_i32(args[3]);

	if (baud < 10 || baud > 10000000) {
		return ENC_SYM_EERROR;
	}

	if (!utils_gpio_is_valid(rx_pin) && !utils_gpio_is_valid(tx_pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	if (uart_num >= UART_NUM_MAX) {
		lbm_set_error_reason("Invalid UART port");
		return ENC_SYM_EERROR;
	}

	m_uart_number = -1;

	xSemaphoreTake(m_uart_mutex, portMAX_DELAY);
	ublox_stop(uart_num);
	comm_uart_stop(uart_num);

	if (uart_is_driver_installed(uart_num)) {
		uart_driver_delete(uart_num);
	}
	xSemaphoreGive(m_uart_mutex);

	uart_config_t uart_config = {
			.baud_rate = baud,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT,
	};
	esp_err_t r;
	r = uart_driver_install(uart_num, 512, 512, 0, 0, 0);
	if (r != ESP_OK) {
		return ENC_SYM_NIL;
	}
	r = uart_param_config(uart_num, &uart_config);
	if (r != ESP_OK) {
		return ENC_SYM_NIL;
	}
	r = uart_set_pin(uart_num, tx_pin, rx_pin, -1, -1);
	if (r != ESP_OK) {
		return ENC_SYM_NIL;
	}

	m_uart_number = uart_num;
	return ENC_SYM_TRUE;
}

static lbm_value ext_uart_stop(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	if (m_uart_number >= 0) {
		ublox_stop(m_uart_number);
		comm_uart_stop(m_uart_number);

		xSemaphoreTake(m_uart_mutex, portMAX_DELAY);
		if (uart_is_driver_installed(m_uart_number)) {
			uart_driver_delete(m_uart_number);
		}
		xSemaphoreGive(m_uart_mutex);
	}

	m_uart_number = -1;

	return ENC_SYM_TRUE;
}

static lbm_value ext_uart_write(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || (!lbm_is_cons(args[0]) && !lbm_is_array_r(args[0]))) {
		return ENC_SYM_EERROR;
	}

	if (m_uart_number < 0) {
		return ENC_SYM_NIL;
	}

	const int max_len = 50;
	uint8_t to_send[max_len];
	uint8_t *to_send_ptr = to_send;
	int ind = 0;

	if (lbm_is_array_r(args[0])) {
		lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
		to_send_ptr = (uint8_t*)array->data;
		ind = array->size;
	} else {
		lbm_value curr = args[0];
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
	// This may take a while for large buffer, while lbm is blocked.
	uart_write_bytes(m_uart_number,to_send_ptr, ind);
	return ENC_SYM_TRUE;
}

typedef struct {
	lbm_cid id;
	unsigned int num;
	unsigned int offset;
	int stop_at;
	TickType_t timeout;
	uint8_t *data;
	bool unblock;
	unsigned int res;
} uart_rx_args;

static void uart_rx_task(void *arg) {
	uart_rx_args *a = (uart_rx_args*)arg;
	int restart_cnt = lispif_get_restart_cnt();

	xSemaphoreTake(m_uart_mutex, portMAX_DELAY);
	unsigned int count = 0;
	uint8_t c;
	int res = uart_read_bytes(m_uart_number, &c, 1, a->timeout);
	while (res == 1) {
		a->data[a->offset + count] = c;
		count++;
		if (c == a->stop_at || count >= a->num) {
			break;
		}
		res = uart_read_bytes(m_uart_number, &c, 1, a->timeout);
	}
	xSemaphoreGive(m_uart_mutex);

	a->res = count;

	if (a->unblock) {
		if (restart_cnt == lispif_get_restart_cnt()) {
			lbm_unblock_ctx_unboxed(a->id, lbm_enc_u(a->res));
		}
		vTaskDelete(NULL);
	}
}

// (uart-read array num optOffset optStopAt optTimeout)
static lbm_value ext_uart_read(lbm_value *args, lbm_uint argn) {
	if ((argn != 2 && argn != 3 && argn != 4 && argn != 5) ||
			!lbm_is_array_r(args[0]) || !lbm_is_number(args[1])) {
		return ENC_SYM_TERROR;
	}

	unsigned int num = lbm_dec_as_u32(args[1]);
	if (num > 512) {
		return ENC_SYM_EERROR;
	}

	if (num == 0 || (m_uart_number < 0)) {
		return lbm_enc_i(0);
	}

	unsigned int offset = 0;
	if (argn >= 3) {
		if (lbm_is_number(args[2])) {
			offset = lbm_dec_as_u32(args[2]);
		}
	}

	int stop_at = -1;
	if (argn >= 4) {
		if (lbm_is_number(args[3])) {
			stop_at = lbm_dec_as_u32(args[3]);
		}
	}

	TickType_t timeout = 0;
	if (argn >= 5) {
		if (lbm_is_number(args[4])) {
			timeout = (TickType_t)(lbm_dec_as_float(args[4]) * (float)portTICK_PERIOD_MS * 1000.0);
		}
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[0]);
	if (array->size < (num + offset)) {
		return ENC_SYM_EERROR;
	}

	static uart_rx_args a;
	a.id = lbm_get_current_cid();
	a.num = num;
	a.offset = offset;
	a.stop_at = stop_at;
	a.timeout = timeout;
	a.data = (uint8_t*)array->data;

	if (timeout == 0) {
		a.unblock = false;
		uart_rx_task(&a);
		return lbm_enc_u(a.res);
	} else {
		a.unblock = true;
		lbm_block_ctx_from_extension();
		xTaskCreatePinnedToCore(uart_rx_task, "Uart Rx", 2048, &a, 7, NULL, tskNO_AFFINITY);
		return ENC_SYM_TRUE;
	}
}

// UARTCOMM

// (uartcomm-start uart-num rx-pin tx-pin baud)
static lbm_value ext_uartcomm_start(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	int uart_num = lbm_dec_as_i32(args[0]);
	int rx_pin = lbm_dec_as_i32(args[1]);
	int tx_pin = lbm_dec_as_i32(args[2]);
	int baud = lbm_dec_as_i32(args[3]);

	if (baud < 10 || baud > 10000000) {
		return ENC_SYM_EERROR;
	}

	if (!utils_gpio_is_valid(rx_pin) && !utils_gpio_is_valid(tx_pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_EERROR;
	}

	if (uart_num >= UART_NUM_MAX) {
		lbm_set_error_reason("Invalid UART port");
		return ENC_SYM_EERROR;
	}

	if (m_uart_number == uart_num) {
		xSemaphoreTake(m_uart_mutex, portMAX_DELAY);
		m_uart_number = -1;
		xSemaphoreGive(m_uart_mutex);
	}

	ublox_stop(uart_num);
	bool res = comm_uart_init(tx_pin, rx_pin, uart_num, baud);

	return res ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (uartcomm-stop uart-num)
static lbm_value ext_uartcomm_stop(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int uart_num = lbm_dec_as_i32(args[0]);

	if (uart_num >= UART_NUM_MAX) {
		lbm_set_error_reason("Invalid UART port");
		return ENC_SYM_EERROR;
	}

	if (m_uart_number == uart_num) {
		xSemaphoreTake(m_uart_mutex, portMAX_DELAY);
		m_uart_number = -1;
		xSemaphoreGive(m_uart_mutex);
	}

	ublox_stop(uart_num);
	comm_uart_stop(uart_num);

	return ENC_SYM_TRUE;
}

// PWM

static int pwm_max[LEDC_TIMER_MAX];
static char *err_invalid_chan = "Invalid channel";

// (pwm-start freq duty channel pin optBits)
static lbm_value ext_pwm_start(lbm_value *args, lbm_uint argn) {
	if (argn != 4 && argn != 5) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	LBM_CHECK_NUMBER_ALL();

	uint32_t freq = lbm_dec_as_u32(args[0]);

	float duty = lbm_dec_as_float(args[1]);
	utils_truncate_number(&duty, 0.0, 1.0);

	uint32_t chan = lbm_dec_as_u32(args[2]);
	if (chan >= LEDC_TIMER_MAX) {
		lbm_set_error_reason(err_invalid_chan);
		return ENC_SYM_TERROR;
	}

	int pin = lbm_dec_as_i32(args[3]);
	if (!utils_gpio_is_valid(pin)) {
		lbm_set_error_reason(string_pin_invalid);
		return ENC_SYM_TERROR;
	}

	int bits = 10;
	if (argn >= 5) {
		bits = lbm_dec_as_u32(args[4]);
		if (bits < 2 || bits > 14) {
			lbm_set_error_reason("Invalid number of bits");
			return ENC_SYM_TERROR;
		}
	}

	pwm_max[chan] = (1 << bits);

	int duty_i = (int)(duty * (float)pwm_max[chan]);

	ledc_timer_config_t ledc_timer = {
			.speed_mode       = LEDC_LOW_SPEED_MODE,
			.timer_num        = chan,
			.duty_resolution  = bits,
			.freq_hz          = freq,
			.clk_cfg          = LEDC_AUTO_CLK
	};

	if (ledc_timer_config(&ledc_timer) != ESP_OK) {
		lbm_set_error_reason("Invalid bit and frequency combination");
		return ENC_SYM_EERROR;
	}

	ledc_channel_config_t ledc_channel = {
			.speed_mode     = LEDC_LOW_SPEED_MODE,
			.channel        = chan,
			.timer_sel      = chan,
			.intr_type      = LEDC_INTR_DISABLE,
			.gpio_num       = pin,
			.duty           = duty_i,
			.hpoint         = 0
	};

	if (ledc_channel_config(&ledc_channel) != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	return lbm_enc_i(ledc_get_freq(LEDC_LOW_SPEED_MODE, chan));
}

// (pwm-stop channel)
static lbm_value ext_pwm_stop(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	uint32_t chan = lbm_dec_as_u32(args[0]);
	if (chan >= LEDC_TIMER_MAX) {
		lbm_set_error_reason(err_invalid_chan);
		return ENC_SYM_TERROR;
	}

	ledc_stop(LEDC_LOW_SPEED_MODE, chan, 0);
	return ENC_SYM_TRUE;
}

// (pwm-set-duty duty channel)
static lbm_value ext_pwm_set_duty(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(2);

	uint32_t chan = lbm_dec_as_u32(args[1]);
	if (chan >= LEDC_TIMER_MAX) {
		lbm_set_error_reason(err_invalid_chan);
		return ENC_SYM_TERROR;
	}

	float duty = lbm_dec_as_float(args[0]);
	utils_truncate_number(&duty, 0.0, 1.0);

	int duty_i = (int)(duty * (float)pwm_max[chan]);

	if (ledc_set_duty(LEDC_LOW_SPEED_MODE, chan, duty_i) != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	if (ledc_update_duty(LEDC_LOW_SPEED_MODE, chan) != ESP_OK) {
		return ENC_SYM_EERROR;
	}

	return lbm_enc_float((float)duty_i / (float)pwm_max[chan]);
}

// Compression

typedef struct {
	FILE *input;
	unsigned int input_length;
	unsigned char input_chunk[256];
	unsigned int input_chunk_start;
	unsigned int input_chunk_end;
} read_file_state;

typedef struct {
	unsigned char *data;
	unsigned int len;
} read_buf_state;

static unsigned int my_lz_read_file(void *udata, unsigned int offset) {
	read_file_state *st = (read_file_state *) udata;

	// Most reads should be cached here with no file I/O.
	if (offset >= st->input_chunk_start && offset < st->input_chunk_end) {
		return (unsigned int) st->input_chunk[offset - st->input_chunk_start];
	}

	// Out-of-bounds read, no file I/O.
	if (offset >= st->input_length) {
		commands_printf_lisp("Unzip: OOB read (offset %ld)\n", (long)offset);
		return 0x100U;
	}

	/* Load in new chunk so that desired offset is in the middle.
	 * This makes backwards and forwards scanning reasonably
	 * efficient.
	 */
	int chunk_start = offset - sizeof(st->input_chunk) / 2;
	if (chunk_start < 0) {
		chunk_start = 0;
	}
	if (fseek(st->input, (size_t) chunk_start, SEEK_SET) != 0) {
		commands_printf_lisp("Unzip: fseek failed");
		return 0x100U;
	}

	size_t got = fread((void *) st->input_chunk, 1, sizeof(st->input_chunk), st->input);
	st->input_chunk_start = chunk_start;
	st->input_chunk_end = chunk_start + got;

	// Recheck original request
	if (offset >= st->input_chunk_start && offset < st->input_chunk_end) {
		return (unsigned int) st->input_chunk[offset - st->input_chunk_start];
	}

	commands_printf_lisp("Unzip: file read error");
	return 0x100U;
}

static unsigned int my_lz_read_buf(void *udata, unsigned int offset) {
	read_buf_state *st = (read_buf_state*)udata;
	if (offset < st->len) {
		return st->data[offset];
	}

	return 0x100;
}

typedef struct {
	unsigned int offset;
	unsigned int buf_offset;
	unsigned char buffer[256];
} write_file_state;

static void my_lz_write(void *udata, int byte) {
	write_file_state *st = (write_file_state *)udata;

	// If byte is negative it means that it is an index in the past of the output
	// from where data should be copied to the front of the output.
	if (byte < 0) {
		byte = -byte;

		if (byte <= st->buf_offset) {
			byte = st->buffer[st->buf_offset - byte];
		} else {
			byte = ((unsigned char*)update_partition_data)[st->offset - (byte - st->buf_offset)];
		}
	}

	st->buffer[st->buf_offset++] = byte;

	if (st->buf_offset == sizeof(st->buffer)) {
		esp_partition_write(update_partition, st->offset, st->buffer, st->buf_offset);
		st->offset += st->buf_offset;
		st->buf_offset = 0;
	}
}

static void my_lz_write_sync(void *udata) {
	write_file_state *st = (write_file_state *)udata;

	if (st->buf_offset > 0) {
		esp_partition_write(update_partition, st->offset, st->buffer, st->buf_offset);
		st->offset += st->buf_offset;
		st->buf_offset = 0;
	}
}

typedef struct {
	lbm_cid id;
	lowzip_state *st;
	read_file_state *st_file;
	read_buf_state *st_buf;
	write_file_state *st_write;
	FILE *f_out;
	unsigned int buflen;
} unzip_args;

static void unzip_task(void *arg) {
	unzip_args *a = (unzip_args*)arg;

	// Use restart counter to tell if LBM has been restarted while this thread
	// was running.
	int restart_cnt = lispif_get_restart_cnt();

	uint32_t erase_len = (a->buflen / update_partition->erase_size) * update_partition->erase_size;
	if ((a->buflen % update_partition->erase_size) != 0) {
		erase_len += update_partition->erase_size;
	}

	esp_partition_erase_range(update_partition, 0, erase_len);

	if (restart_cnt == lispif_get_restart_cnt()) {
		lowzip_get_data(a->st);
	}

	if (restart_cnt == lispif_get_restart_cnt()) {
		lbm_value res = ENC_SYM_NIL;
		if (!a->st->have_error) {
			unsigned int count = fwrite(update_partition_data, 1, a->buflen, a->f_out);
			fsync(fileno(a->f_out));
			res = count == a->buflen ? ENC_SYM_TRUE : ENC_SYM_NIL;

			if (!res) {
				commands_printf_lisp("Unzip: could not write all data to output file");
			}
		} else {
			commands_printf_lisp("Unzip: get_data error in extension");
		}

		lbm_free(a->st);
		lbm_free(a->st_file);
		lbm_free(a->st_buf);
		lbm_free(a->st_write);
		lbm_free(a);

		lbm_unblock_ctx_unboxed(a->id, res);
	}

	vTaskDelete(NULL);
}

// (unzip input fileInZip optOutputFile)
static lbm_value ext_unzip(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		return ENC_SYM_TERROR;
	}

	FILE *f_in = NULL;
	lbm_array_header_t *arr_in = NULL;
	if (lbm_is_number(args[0])) {
		f_in = file_from_arg(args[0]);
		if (!f_in) {
			lbm_set_error_reason((char*)str_f_not_open);
			return ENC_SYM_EERROR;
		}
	} else if (lbm_is_array_r(args[0])) {
		arr_in = (lbm_array_header_t *)lbm_car(args[0]);
	} else {
		return ENC_SYM_TERROR;
	}

	int ind_in_zip = 0;
	char *name_in_zip = NULL;
	if (lbm_is_number(args[1])) {
		ind_in_zip = lbm_dec_as_i32(args[1]);
		if (ind_in_zip < 0) {
			return ENC_SYM_TERROR;
		}
	} else {
		name_in_zip = lbm_dec_str(args[1]);
		if (!name_in_zip) {
			return ENC_SYM_TERROR;
		}
	}

	FILE *f_out = NULL;
	if (argn == 3) {
		f_out = file_from_arg(args[2]);
		if (!f_out) {
			lbm_set_error_reason((char*)str_f_not_open);
			return ENC_SYM_EERROR;
		}
	}

	lowzip_state *st = NULL;
	st = (lowzip_state *) lbm_malloc(sizeof(lowzip_state));
	if (!st) {
		return ENC_SYM_MERROR;
	}

	memset((void *) st, 0, sizeof(lowzip_state));

	read_file_state *st_file = NULL;
	read_buf_state *st_buf = NULL;

	if (f_in) {
		st_file = (read_file_state*)lbm_malloc(sizeof(read_file_state));
		if (!st_file) {
			lbm_free(st);
			return ENC_SYM_MERROR;
		}
		memset((void *)st_file, 0, sizeof(read_file_state));

		fseek(f_in, 0, SEEK_END);

		st_file->input = f_in;
		st_file->input_length = ftell(f_in);

		st->udata = (void *)st_file;
		st->read_callback = my_lz_read_file;
		st->zip_length = st_file->input_length;
	} else {
		st_buf = (read_buf_state*)lbm_malloc(sizeof(read_buf_state));
		if (!st_buf) {
			lbm_free(st);
			return ENC_SYM_MERROR;
		}
		memset((void *)st_buf, 0, sizeof(read_buf_state));

		st_buf->data = (unsigned char*)arr_in->data;
		st_buf->len = arr_in->size;

		st->udata = st_buf;
		st->read_callback = my_lz_read_buf;
		st->zip_length = st_buf->len;
	}

	lowzip_init_archive(st);

	if (st->have_error) {
		lbm_set_error_reason("Invalid zip archive");
		lbm_free(st);
		lbm_free(st_file);
		lbm_free(st_buf);
		return ENC_SYM_EERROR;
	}

	lowzip_file *fileinfo = lowzip_locate_file(st, ind_in_zip, name_in_zip);
	if (!fileinfo) {
		lbm_set_error_reason("Invalid file in zip");
		lbm_free(st);
		lbm_free(st_file);
		lbm_free(st_buf);
		return ENC_SYM_EERROR;
	}

	if (f_out) {
		write_file_state *st_write = lbm_malloc(sizeof(write_file_state));
		if (!st_write) {
			lbm_free(st);
			lbm_free(st_file);
			lbm_free(st_buf);
			return ENC_SYM_MERROR;
		}

		memset((void *)st_write, 0, sizeof(write_file_state));

		if (!fw_map_buffer()) {
			lbm_free(st);
			lbm_free(st_file);
			lbm_free(st_buf);
			lbm_free(st_write);
			return ENC_SYM_EERROR;
		}

		unsigned int buflen = fileinfo->uncompressed_size;

		if (buflen > update_partition->size) {
			update_partition = NULL;
			lbm_free(st);
			lbm_free(st_file);
			lbm_free(st_buf);
			lbm_free(st_write);
			lbm_set_error_reason("Too large file");
			return ENC_SYM_EERROR;
		}

		st->output_start = (unsigned char *)update_partition_data;
		st->output_end = st->output_start + buflen;
		st->output_next = st->output_start;

		st->write_callback = my_lz_write;
		st->write_sync_callback = my_lz_write_sync;
		st->udata_write = st_write;

		unzip_args *a;
		a = (unzip_args*)lbm_malloc(sizeof(unzip_args));

		if (!a) {
			lbm_free(st);
			lbm_free(st_file);
			lbm_free(st_buf);
			lbm_free(st_write);
			return ENC_SYM_MERROR;
		}

		a->id = lbm_get_current_cid();
		a->st = st;
		a->st_file = st_file;
		a->st_buf = st_buf;
		a->st_write = st_write;
		a->f_out = f_out;
		a->buflen = buflen;

		xTaskCreatePinnedToCore(unzip_task, "Unzip", 3072, a, 5, NULL, tskNO_AFFINITY);

		lbm_block_ctx_from_extension();
		return ENC_SYM_TRUE;
	} else {
		lbm_value res = ENC_SYM_MERROR;
		if (lbm_create_array(&res, fileinfo->uncompressed_size)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);

			st->output_start = (unsigned char *)arr->data;
			st->output_end = st->output_start + fileinfo->uncompressed_size;
			st->output_next = st->output_start;

			lowzip_get_data(st);

			if (st->have_error) {
				commands_printf_lisp("Unzip: get_data error in extension");
				res = ENC_SYM_NIL;
			}
		}

		lbm_free(st);
		lbm_free(st_file);
		lbm_free(st_buf);
		return res;
	}
}

// (zip-ls input)
static lbm_value ext_zip_ls(lbm_value *args, lbm_uint argn) {
	if (argn != 1) {
		return ENC_SYM_TERROR;
	}

	FILE *f_in = NULL;
	lbm_array_header_t *arr_in = NULL;
	if (lbm_is_number(args[0])) {
		f_in = file_from_arg(args[0]);
		if (!f_in) {
			lbm_set_error_reason((char*)str_f_not_open);
			return ENC_SYM_EERROR;
		}
	} else if (lbm_is_array_r(args[0])) {
		arr_in = (lbm_array_header_t *)lbm_car(args[0]);
	} else {
		return ENC_SYM_TERROR;
	}

	lowzip_state *st = NULL;
	st = (lowzip_state *) lbm_malloc(sizeof(lowzip_state));
	if (!st) {
		return ENC_SYM_MERROR;
	}

	memset((void *) st, 0, sizeof(lowzip_state));

	read_file_state *st_file = NULL;
	read_buf_state st_buf;

	if (f_in) {
		st_file = (read_file_state *) lbm_malloc(sizeof(read_file_state));
		if (!st_file) {
			lbm_free(st);
			return ENC_SYM_MERROR;
		}

		memset((void *)st_file, 0, sizeof(read_file_state));
		fseek(f_in, 0, SEEK_END);

		st_file->input = f_in;
		st_file->input_length = ftell(f_in);

		st->udata = (void *)st_file;
		st->read_callback = my_lz_read_file;
		st->zip_length = st_file->input_length;
	} else {
		st_buf.data = (unsigned char*)arr_in->data;
		st_buf.len = arr_in->size;

		st->udata = (void *)&st_buf;
		st->read_callback = my_lz_read_buf;
		st->zip_length = st_buf.len;
	}

	lowzip_init_archive(st);

	if (st->have_error) {
		lbm_set_error_reason("Invalid zip archive");
		lbm_free(st);
		lbm_free(st_file);
		return ENC_SYM_EERROR;
	}

	lbm_value r = ENC_SYM_NIL;

	for (int i = 0; ; i++) {
		lowzip_file *fileinfo = lowzip_locate_file(st, i, NULL);
		if (!fileinfo) {
			break;
		}

		lbm_value current = ENC_SYM_NIL;
		current = lbm_cons(lbm_enc_i(fileinfo->uncompressed_size), current);

		lbm_value filename;
		if(lbm_create_array(&filename, (lbm_uint)strlen(fileinfo->filename) + 1)){
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(filename);
			strcpy((char*)arr->data, fileinfo->filename);
			current = lbm_cons(filename, current);
		} else {
			lbm_free(st);
			lbm_free(st_file);
			return ENC_SYM_MERROR;
		}

		r = lbm_cons(current, r);
	}

	lbm_free(st);
	lbm_free(st_file);
	return r;
}

// Connection checks

static lbm_value ext_connected_wifi(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return comm_wifi_is_client_connected() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_connected_hub(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return comm_wifi_is_connected_hub() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_connected_ble(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return comm_ble_is_connected() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_connected_usb(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return usb_serial_jtag_is_connected() ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

// Crypto

// (aes-ctr-crypt key counter data start-offset length) -> t
static lbm_value ext_aes_ctr_crypt(lbm_value *args, lbm_uint argn) {
    if (!lbm_check_argn_range(argn, 5, 5)) {
        return ENC_SYM_EERROR;
    }
    if (!lbm_is_array_r(args[0]) || !lbm_is_array_rw(args[1]) || !lbm_is_array_rw(args[2])) {
        lbm_set_error_reason("Invalid array types");
        return ENC_SYM_EERROR;
    }

    lbm_array_header_t *key_header     = (lbm_array_header_t *)lbm_car(args[0]);
    lbm_array_header_t *counter_header = (lbm_array_header_t *)lbm_car(args[1]);
    lbm_array_header_t *data_header    = (lbm_array_header_t *)lbm_car(args[2]);

    size_t key_len = key_header->size;
    if (!(key_len == 16 || key_len == 24 || key_len == 32)) {
        lbm_set_error_reason("Key must be 16, 24, or 32 bytes");
        return ENC_SYM_EERROR;
    }
    if (counter_header->size != 16) {
        lbm_set_error_reason("Counter must be 16 bytes");
        return ENC_SYM_EERROR;
    }

    lbm_int start_offset = lbm_dec_as_i32(args[3]);
    lbm_int length       = lbm_dec_as_i32(args[4]);
    if (start_offset < 0 || length < 0 ||
        (start_offset + length) > (lbm_int)data_header->size) {
        lbm_set_error_reason("Invalid start/length");
        return ENC_SYM_EERROR;
    }

    int rc = aes_ctr_crypt_inplace(
        (const uint8_t*)key_header->data, key_len,
        (uint8_t*)counter_header->data,
        (uint8_t*)data_header->data,
        (size_t)start_offset,
        (size_t)length
    );

    if (rc != 0) {
        lbm_set_error_reason("AES-CTR failed");
        return ENC_SYM_EERROR;
    }
    return ENC_SYM_TRUE;
}

// NVS

static char* NVS_DEFAULT_NS = "lbm-nvs";
static char* NVS_QML_NS = "lbm";

// (nvs_qml_erase) -> t
static lbm_value ext_nvs_qml_erase_partition(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	esp_err_t ret = nvs_flash_erase_partition("qml");
	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

// (nvs_qml_init) -> t
static lbm_value ext_nvs_qml_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	esp_err_t ret = nvs_flash_init_partition("qml");
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase_partition("qml");
		ret = nvs_flash_init_partition("qml");
	}

	return ret == ESP_OK ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value nvs_read(char *partition, char *namespace, char *key) {

	nvs_handle_t my_handle;
	esp_err_t ret = nvs_open_from_partition(partition, namespace, NVS_READONLY, &my_handle);
	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not open partition");
		return ENC_SYM_EERROR;
	}

	size_t required_size = 0;
	ret = nvs_get_blob(my_handle, key, NULL, &required_size);
	if (ret == ESP_ERR_NVS_NOT_FOUND){
		nvs_close(my_handle);
		return ENC_SYM_NIL;
	}
	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not read data");
		nvs_close(my_handle);
		return ENC_SYM_EERROR;
	}

	lbm_value res;
	if (lbm_create_array(&res, required_size)) {
		lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(res);
		nvs_get_blob(my_handle, key, arr->data, &required_size);
		nvs_close(my_handle);
		return res;
	} else {
		nvs_close(my_handle);
		return ENC_SYM_MERROR;
	}
}

char* dec_nvs_key(lbm_value v){
	char *key = dec_str_check(v);
	if (key && strlen(key) > 14) {
		lbm_set_error_reason("Invalid key length, must be < 15 characters");
		return 0;
	}
	return key;
}

// (nvs_read key) -> t
static lbm_value ext_nvs_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	char *key = dec_nvs_key(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}
	return nvs_read("nvs", NVS_DEFAULT_NS, key);
}

// (nvs_qml_read key) -> t
static lbm_value ext_nvs_qml_read(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	char *key = dec_nvs_key(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}
	return nvs_read("qml", NVS_QML_NS, key);
}

static lbm_value nvs_write(char *partition, char *namespace, char *key, lbm_array_header_t *array){
	nvs_handle_t my_handle;
	esp_err_t ret = nvs_open_from_partition(partition, namespace, NVS_READWRITE, &my_handle);
	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not open partition");
		return ENC_SYM_EERROR;
	}

	ret = nvs_set_blob(my_handle, key, (void*)array->data, array->size);
	if (ret == ESP_OK) {
		ret = nvs_commit(my_handle);
	}

	nvs_close(my_handle);

	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not write data");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

// (nvs_write key data) -> t
static lbm_value ext_nvs_write(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	char *key = dec_nvs_key(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}
	
	if (!lbm_is_array_r(args[1])) {
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);

	return nvs_write("nvs", NVS_DEFAULT_NS, key, array);
}

// (nvs_qml_write key data) -> t
static lbm_value ext_nvs_qml_write(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(2);

	char *key = dec_nvs_key(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}

	if (!lbm_is_array_r(args[1])) {
		lbm_set_error_reason((char*)lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	lbm_array_header_t *array = (lbm_array_header_t *)lbm_car(args[1]);

	return nvs_write("qml", NVS_QML_NS, key, array);
}

static lbm_value nvs_erase_ns(char *partition, char *namespace){
	nvs_handle_t nvs_handle;
	esp_err_t ret = nvs_open_from_partition(partition, namespace, NVS_READWRITE, &nvs_handle);
	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not open partition");
		return ENC_SYM_EERROR;
	}
	ret = nvs_erase_all(nvs_handle);
	if (ret == ESP_OK) {
		ret = nvs_commit(nvs_handle);
	}

	nvs_close(nvs_handle);

	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not erase keys");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

static lbm_value nvs_erase(char *partition, char *namespace, char *key){
	nvs_handle_t my_handle;
	esp_err_t ret = nvs_open_from_partition(partition, namespace, NVS_READWRITE, &my_handle);
	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not open partition");
		return ENC_SYM_EERROR;
	}

	ret = nvs_erase_key(my_handle, key);
	if (ret == ESP_OK) {
		ret = nvs_commit(my_handle);
	}

	nvs_close(my_handle);

	if (ret != ESP_OK) {
		lbm_set_error_reason("Could not erase key");
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

// (nvs_qml_erase_key key) -> t
static lbm_value ext_nvs_qml_erase_key(lbm_value *args, lbm_uint argn) {
	if (argn == 0){
		return nvs_erase_ns("qml", NVS_QML_NS);
	}
	
	LBM_CHECK_ARGN(1);

	char *key = dec_nvs_key(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}
	
	return nvs_erase("qml", NVS_QML_NS, key);
}

static lbm_value ext_nvs_erase(lbm_value *args, lbm_uint argn){
	if (argn == 0){
		return nvs_erase_ns("nvs", NVS_DEFAULT_NS);
	}
	
	LBM_CHECK_ARGN(1);

	char *key = dec_str_check(args[0]);
	if (!key) {
		return ENC_SYM_TERROR;
	}
	return nvs_erase("nvs", NVS_DEFAULT_NS, key);
}

static lbm_value nvs_list(char *partition, char *namespace, nvs_type_t type) {
	nvs_iterator_t it = NULL;
	esp_err_t esp_res = nvs_entry_find(partition, namespace, type, &it);
	lbm_value res = ENC_SYM_NIL;

	while (esp_res == ESP_OK) {
		nvs_entry_info_t info;
		nvs_entry_info(it, &info);

		lbm_value tok;
		int len = strnlen(info.key, NVS_KEY_NAME_MAX_SIZE);

		if (lbm_create_array(&tok, len + 1)) {
			lbm_array_header_t *arr = (lbm_array_header_t*)lbm_car(tok);
			memcpy(arr->data, info.key, len);
			((char*)(arr->data))[len] = '\0';
			res = lbm_cons(tok, res);
		} else {
			res = ENC_SYM_MERROR;
			break;
		}

		esp_res = nvs_entry_next(&it);
	}

	nvs_release_iterator(it);

	return res;
}

lbm_value ext_image_save(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	bool r = lbm_image_save_global_env();

	lbm_uint main_sym = ENC_SYM_NIL;
	if (lbm_get_symbol_by_name("main", &main_sym)) {
		lbm_value binding;
		if (lbm_global_env_lookup(&binding, lbm_enc_sym(main_sym))) {
			if (lbm_is_cons(binding) && lbm_car(binding) == ENC_SYM_CLOSURE) {
				goto image_has_main;
			}
		}
	}

	lbm_set_error_reason("No main function in image\n");
	return ENC_SYM_EERROR;

	image_has_main:
	r = r && lbm_image_save_extensions();
	r = r && lbm_image_save_constant_heap_ix();
	return r ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_nvs_qml_list(lbm_value *args, lbm_uint argn) {
   return nvs_list("qml", NVS_QML_NS, NVS_TYPE_BLOB);
}

static lbm_value ext_nvs_list(lbm_value *args, lbm_uint argn) {
   return nvs_list("nvs",NVS_DEFAULT_NS, NVS_TYPE_BLOB);
}

// Commands interface
typedef struct {
	PACKET_STATE_t cmds_packet_state;
	void *cmds_thd_stack;
	size_t cmds_thd_stack_size;
	unsigned char buffer[PACKET_MAX_PL_LEN + 8];
	unsigned int len;
} cmds_send_data;

static cmds_send_data *cmds_state = 0;
static StaticTask_t cmds_thd;
static volatile bool cmds_running = false;

static void cmds_send_raw(unsigned char *buffer, unsigned int len) {
	if (event_cmds_data_tx_en) {
		int restart_cnt = lispif_get_restart_cnt();

		lbm_flat_value_t v;
		if (start_flatten_with_gc(&v, len + 30)) {
			f_cons(&v);
			f_sym(&v, sym_event_cmds_data_tx);
			f_cons(&v);
			f_lbm_array(&v, len, buffer);
			f_sym(&v, ENC_SYM_NIL);
			lbm_finish_flatten(&v);

			int timeout = 500;
			while (!lbm_event(&v)) {
				if (restart_cnt != lispif_get_restart_cnt()) {
					return;
				}

				if (timeout == 0 || lispif_is_eval_task() || !event_cmds_data_tx_en) {
					lbm_free(v.buf);
					return;
				}

				vTaskDelay(1);
				timeout--;
			}
		}
	}
}

static void cmds_send_packet(unsigned char *buffer, unsigned int len) {
	if (cmds_state) {
		packet_send_packet(buffer, len, &(cmds_state->cmds_packet_state));
	}
}

static void cmds_send_task(void *arg) {
	(void)arg;
	commands_process_packet(cmds_state->buffer, cmds_state->len, cmds_send_packet);
	vTaskDelete(NULL);
}

static void cmds_send_task_del(int a, void *arg) {
	(void)a;
	(void)arg;
	cmds_running = false;
}

static void cmds_proc(unsigned char *data, unsigned int len) {
	// 10 ms timeout. TODO: Block only task and not all of eval when waiting
	int timeout = 10;
	while (cmds_running) {
		vTaskDelay(1);
		timeout--;
		if (timeout == 0) {
			return;
		}
	}

	if (!cmds_state) {
		return;
	}

	memcpy(cmds_state->buffer, data, len);
	cmds_state->len = len;

	cmds_running = true;
	TaskHandle_t task = xTaskCreateStaticPinnedToCore(
		cmds_send_task,
		"lbm_cmds",
		cmds_state->cmds_thd_stack_size,
		0,
		8,
		cmds_state->cmds_thd_stack,
		&cmds_thd,
		tskNO_AFFINITY
	);

	vTaskSetThreadLocalStoragePointerAndDelCallback(task, 0, 0, cmds_send_task_del);
}

static lbm_value ext_cmds_start_stop(lbm_value *args, lbm_uint argn) {
	if (argn != 0 && argn != 1) {
		lbm_set_error_reason(lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	bool start = true;
	if (argn >= 1) {
		if (!is_symbol_true_false(args[0])) {
			return ENC_SYM_TERROR;
		}

		start = lbm_is_symbol_true(args[0]);
	}

	while (cmds_running) {
		vTaskDelay(1);
	}

	cmds_running = false;

	if (cmds_state) {
		lbm_free(cmds_state->cmds_thd_stack);
		lbm_free(cmds_state);
		cmds_state = 0;
	}

	if (start) {
		cmds_state = lbm_malloc(sizeof(cmds_send_data));

		if (!cmds_state) {
			return ENC_SYM_MERROR;
		}

		cmds_state->cmds_thd_stack = lbm_malloc(3500);
		cmds_state->cmds_thd_stack_size = 3500;

		if (!cmds_state->cmds_thd_stack) {
			lbm_free(cmds_state);
			cmds_state = 0;
			return ENC_SYM_MERROR;
		}

		packet_init(cmds_send_raw, cmds_proc, &(cmds_state->cmds_packet_state));
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_cmds_proc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN(1);

	lbm_array_header_t *arr = lbm_dec_array_header(args[0]);
	if (!arr) {
		lbm_set_error_reason(lbm_error_str_incorrect_arg);
		return ENC_SYM_TERROR;
	}

	if (!cmds_state) {
		lbm_set_error_reason("Cmds not started");
		return ENC_SYM_EERROR;
	}

	for (unsigned int i = 0;i < arr->size;i++) {
		packet_process_byte(((unsigned char *)arr->data)[i],&(cmds_state->cmds_packet_state));
	}

	return ENC_SYM_TRUE;
}

static const char* dyn_functions[] = {
		"(defun uart-read-bytes (buffer n ofs)"
		"(let ((rd (uart-read buffer n ofs)))"
		"(if (= rd n)"
		"(bufset-u8 buffer (+ ofs rd) 0)"
		"(progn (yield 4000) (uart-read-bytes buffer (- n rd) (+ ofs rd)))"
		")))",

		"(defun uart-read-until (buffer n ofs end)"
		"(let ((rd (uart-read buffer n ofs end)))"
		"(if (and (> rd 0) (or (= rd n) (= (bufget-u8 buffer (+ ofs (- rd 1))) end)))"
		"(bufset-u8 buffer (+ ofs rd) 0)"
		"(progn (yield 10000) (uart-read-until buffer (- n rd) (+ ofs rd) end))"
		")))",

		"(defun img-buffer-from-bin (x) x)",
};

static bool dynamic_loader(const char *str, const char **code) {
	for (unsigned int i = 0; i < (sizeof(dyn_functions) / sizeof(dyn_functions[0]));i++) {
		if (strmatch(str, dyn_functions[i] + 7)) {
			*code = dyn_functions[i];
			return true;
		}
	}

	return lbm_dyn_lib_find(str, code);
}

void lispif_load_vesc_extensions(bool main_found) {
	lispif_stop_lib();
	if (!i2c_mutex_init_done) {
		i2c_mutex = xSemaphoreCreateMutex();
		i2c_mutex_init_done = true;
	}

	if (!uart_mutex_init_done) {
		m_uart_mutex = xSemaphoreCreateMutex();
		uart_mutex_init_done = true;
	}

	if (!event_task_running) {
		rmsg_mutex = xSemaphoreCreateMutex();

		xSemaphoreTake(rmsg_mutex, portMAX_DELAY);
		for (int i = 0;i < RMSG_SLOT_NUM;i++) {
			rmsg_slots[i].cid = -1;
			rmsg_slots[i].timeout_secs = -1.0;
		}
		xSemaphoreGive(rmsg_mutex);

		xTaskCreatePinnedToCore(event_task, "LBM Events", 640, NULL, 7, NULL, tskNO_AFFINITY);
		event_task_running = true;
	}

	memset(&syms_vesc, 0, sizeof(syms_vesc));

	if (!main_found) {
		lbm_add_symbol_const("hw-express", &sym_hw_express);
		lbm_add_symbol_const("size", &sym_size);
		lispif_events_load_symbols();

		// Various commands
		lbm_add_extension("print", ext_print);
		lbm_add_extension("set-print-prefix", ext_set_print_prefix);
		lbm_add_extension("set-fw-name", ext_set_fw_name);
		lbm_add_extension("puts", ext_puts);
		lbm_add_extension("get-bms-val", ext_get_bms_val);
		lbm_add_extension("set-bms-val", ext_set_bms_val);
		lbm_add_extension("send-bms-can", ext_send_bms_can);
		lbm_add_extension("set-bms-chg-allowed", ext_set_bms_chg_allowed);
		lbm_add_extension("bms-force-balance", ext_bms_force_balance);
		lbm_add_extension("bms-zero-offset", ext_bms_zero_offset);
		lbm_add_extension("bms-st", ext_bms_st);
		lbm_add_extension("get-adc", ext_get_adc);
		lbm_add_extension("systime", ext_systime);
		lbm_add_extension("secs-since", ext_secs_since);
		lbm_add_extension("event-enable", ext_enable_event);
		lbm_add_extension("send-data", ext_send_data);
		lbm_add_extension("recv-data", ext_recv_data);
		lbm_add_extension("sysinfo", ext_sysinfo);
		lbm_add_extension("import", ext_empty);
		lbm_add_extension("main-init-done", ext_main_init_done);
		lbm_add_extension("crc16", ext_crc16);
		lbm_add_extension("crc32", ext_crc32);
		lbm_add_extension("buf-resize", ext_buf_resize);

		// Configuration
		lbm_add_extension("conf-get", ext_conf_get);
		lbm_add_extension("conf-set", ext_conf_set);
		lbm_add_extension("conf-store", ext_conf_store);
		lbm_add_extension("reboot", ext_reboot);

		// EEPROM
		lbm_add_extension("eeprom-store-f", ext_eeprom_store_f);
		lbm_add_extension("eeprom-read-f", ext_eeprom_read_f);
		lbm_add_extension("eeprom-store-i", ext_eeprom_store_i);
		lbm_add_extension("eeprom-read-i", ext_eeprom_read_i);
		lbm_add_extension("eeprom-erase", ext_eeprom_erase);

		// CAN-comands
		lbm_add_extension("can-start", ext_can_start);
		lbm_add_extension("can-stop", ext_can_stop);
		lbm_add_extension("can-use-vesc", ext_can_use_vesc);
		lbm_add_extension("can-scan", ext_can_scan);
		lbm_add_extension("can-ping", ext_can_ping);
		lbm_add_extension("can-send-sid", ext_can_send_sid);
		lbm_add_extension("can-send-eid", ext_can_send_eid);
		lbm_add_extension("can-recv-sid", ext_can_recv_sid);
		lbm_add_extension("can-recv-eid", ext_can_recv_eid);
		lbm_add_extension("can-cmd", ext_can_cmd);
		lbm_add_extension("can-list-devs", ext_can_list_devs);
		lbm_add_extension("can-local-id", ext_can_local_id);
		lbm_add_extension("can-update-baud", ext_can_update_baud);

		lbm_add_extension("can-msg-age", ext_can_msg_age);
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
		lbm_add_extension("i2c-detect-addr", ext_i2c_detect_addr);

		// GPIO
		lbm_add_extension("gpio-configure", ext_gpio_configure);
		lbm_add_extension("gpio-write", ext_gpio_write);
		lbm_add_extension("gpio-read", ext_gpio_read);
		lbm_add_extension("gpio-hold", ext_gpio_hold);
		lbm_add_extension("gpio-hold-deepsleep", ext_gpio_hold_deepsleep);

		// Math
		lbm_add_extension("throttle-curve", ext_throttle_curve);
		lbm_add_extension("rand", ext_rand);
		lbm_add_extension("rand-max", ext_rand_max);

		// Bit operations
		lbm_add_extension("bits-enc-int", ext_bits_enc_int);
		lbm_add_extension("bits-dec-int", ext_bits_dec_int);

		// Lbm settings
		lbm_add_extension("lbm-set-quota", ext_lbm_set_quota);
		lbm_add_extension("lbm-set-gc-stack-size", ext_lbm_set_gc_stack_size);

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
		lbm_add_extension("esp-now-del-peer", ext_esp_now_del_peer);
		lbm_add_extension("esp-now-send", ext_esp_now_send);
		lbm_add_extension("esp-now-recv", ext_esp_now_recv);
		lbm_add_extension("get-mac-addr", ext_get_mac_addr);
		lbm_add_extension("wifi-get-chan", ext_wifi_get_chan);
		lbm_add_extension("wifi-set-chan", ext_wifi_set_chan);
		lbm_add_extension("wifi-get-bw", ext_wifi_get_bw);
		lbm_add_extension("wifi-set-bw", ext_wifi_set_bw);
		lbm_add_extension("wifi-start", ext_wifi_start);
		lbm_add_extension("wifi-stop", ext_wifi_stop);

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
		lbm_add_extension("ublox-init", ext_ublox_init);
		lbm_add_extension("nmea-parse", ext_nmea_parse);
		lbm_add_extension("set-pos-time", ext_set_pos_time);

		// Sleep
		lbm_add_extension("sleep-deep", ext_sleep_deep);
		lbm_add_extension("sleep-light", ext_sleep_light);
		lbm_add_extension("sleep-config-wakeup-pin", ext_sleep_config_wakeup_pin);
		lbm_add_extension("rtc-data", ext_rtc_data);

		// Native libraries
		lbm_add_extension("load-native-lib", ext_load_native_lib);
		lbm_add_extension("unload-native-lib", ext_unload_native_lib);

		lispif_load_rgbled_extensions();

		lispif_load_disp_extensions();
		lispif_load_wifi_extensions();

		if (backup.config.ble_mode == BLE_MODE_SCRIPTING) {
			lispif_load_ble_extensions();
		}

		// CAN-Messages
		lbm_add_extension("canmsg-recv", ext_canmsg_recv);
		lbm_add_extension("canmsg-send", ext_canmsg_send);

		// File System
		lbm_add_extension("f-connect", ext_f_connect);
		lbm_add_extension("f-connect-nand", ext_f_connect_nand);
		lbm_add_extension("f-disconnect", ext_f_disconnect);
		lbm_add_extension("f-open", ext_f_open);
		lbm_add_extension("f-close", ext_f_close);
		lbm_add_extension("f-read", ext_f_read);
		lbm_add_extension("f-readline", ext_f_readline);
		lbm_add_extension("f-write", ext_f_write);
		lbm_add_extension("f-tell", ext_f_tell);
		lbm_add_extension("f-seek", ext_f_seek);
		lbm_add_extension("f-mkdir", ext_f_mkdir);
		lbm_add_extension("f-rm", ext_f_rm);
		lbm_add_extension("f-ls", ext_f_ls);
		lbm_add_extension("f-size", ext_f_size);
		lbm_add_extension("f-rename", ext_f_rename);
		lbm_add_extension("f-sync", ext_f_sync);
		lbm_add_extension("f-fatinfo", ext_f_fatinfo);

		// Firmware update
		lbm_add_extension("fw-erase", ext_fw_erase);
		lbm_add_extension("fw-write", ext_fw_write);
		lbm_add_extension("fw-reboot", ext_fw_reboot);
		lbm_add_extension("fw-data", ext_fw_data);
		lbm_add_extension("fw-write-raw", ext_fw_write_raw);
		lbm_add_extension("fw-info", ext_fw_info);

		// Lbm and script update
		lbm_add_extension("lbm-erase", ext_lbm_erase);
		lbm_add_extension("lbm-write", ext_lbm_write);
		lbm_add_extension("lbm-run", ext_lbm_run);
		lbm_add_extension("qml-erase", ext_qml_erase);
		lbm_add_extension("qml-write", ext_qml_write);

		// AS504x
		lbm_add_extension("as504x-init", ext_as504x_init);
		lbm_add_extension("as504x-deinit", ext_as504x_deinit);
		lbm_add_extension("as504x-angle", ext_as504x_angle);

		// IMU
		lbm_add_extension("imu-start-lsm6", ext_imu_start_lsm6);
		lbm_add_extension("imu-stop", ext_imu_stop);
		lbm_add_extension("get-imu-rpy", ext_get_imu_rpy);
		lbm_add_extension("get-imu-quat", ext_get_imu_quat);
		lbm_add_extension("get-imu-acc", ext_get_imu_acc);
		lbm_add_extension("get-imu-gyro", ext_get_imu_gyro);
		lbm_add_extension("get-imu-mag", ext_get_imu_mag);
		lbm_add_extension("get-imu-acc-derot", ext_get_imu_acc_derot);
		lbm_add_extension("get-imu-gyro-derot", ext_get_imu_gyro_derot);

		// UART
		lbm_add_extension("uart-start", ext_uart_start);
		lbm_add_extension("uart-stop", ext_uart_stop);
		lbm_add_extension("uart-write", ext_uart_write)	;
		lbm_add_extension("uart-read", ext_uart_read);

		// UARTCOMM
		lbm_add_extension("uartcomm-start", ext_uartcomm_start);
		lbm_add_extension("uartcomm-stop", ext_uartcomm_stop);

		// PWM
		lbm_add_extension("pwm-start", ext_pwm_start);
		lbm_add_extension("pwm-stop", ext_pwm_stop);
		lbm_add_extension("pwm-set-duty", ext_pwm_set_duty);

		// Compression
		lbm_add_extension("unzip", ext_unzip);
		lbm_add_extension("zip-ls", ext_zip_ls);

		// Connection checks
		lbm_add_extension("connected-wifi", ext_connected_wifi);
		lbm_add_extension("connected-hub", ext_connected_hub);
		lbm_add_extension("connected-ble", ext_connected_ble);
		lbm_add_extension("connected-usb", ext_connected_usb);

		// Crypto
		lbm_add_extension("aes-ctr-crypt", ext_aes_ctr_crypt);

		// NVS
		lbm_add_extension("nvs-read", ext_nvs_read); 
		lbm_add_extension("nvs-write", ext_nvs_write);
		lbm_add_extension("nvs-erase", ext_nvs_erase);
		lbm_add_extension("nvs-list", ext_nvs_list);
		
		lbm_add_extension("nvs-qml-erase-partition", ext_nvs_qml_erase_partition); 
		lbm_add_extension("nvs-qml-init", ext_nvs_qml_init);
		lbm_add_extension("nvs-qml-read", ext_nvs_qml_read);
		lbm_add_extension("nvs-qml-write", ext_nvs_qml_write);
		lbm_add_extension("nvs-qml-erase", ext_nvs_qml_erase_key);
		lbm_add_extension("nvs-qml-list", ext_nvs_qml_list);

		// Image
		lbm_add_extension("image-save", ext_image_save);

		// Commands
		lbm_add_extension("cmds-start-stop", ext_cmds_start_stop);
		lbm_add_extension("cmds-proc", ext_cmds_proc);

		// Extension libraries
		lbm_math_extensions_init();
		lbm_color_extensions_init();
		lbm_mutex_extensions_init();
		lbm_ttf_extensions_init();
		lbm_dyn_lib_init();
		lbm_array_extensions_init();
		lbm_string_extensions_init();
	}

	lbm_set_dynamic_load_callback(dynamic_loader);
}

void lispif_disable_all_events(void) {
	if (event_task_running) {
		xSemaphoreTake(rmsg_mutex, portMAX_DELAY);
		for (int i = 0;i < RMSG_SLOT_NUM;i++) {
			rmsg_slots[i].cid = -1;
			rmsg_slots[i].timeout_secs = -1.0;
		}
		xSemaphoreGive(rmsg_mutex);
	}

	lispif_stop_lib();

	event_can_sid_en = false;
	event_can_eid_en = false;
	event_data_rx_en = false;
	event_esp_now_rx_en = false;
	event_ble_rx_en = false;
	event_wifi_disconnect_en = false;
	event_cmds_data_tx_en = false;

	event_bms_chg_allow_en = false;
	event_bms_bal_ovr_en = false;
	event_bms_reset_cnt_en = false;
	event_bms_force_bal_en = false;
	event_bms_zero_ofs_en = false;

	bms_register_cmd_handler(NULL);

	esp_now_recv_cid = -1;
	can_recv_sid_cid = -1;
	can_recv_eid_cid = -1;
	recv_data_cid = -1;

	for (int i = 0;i < file_now;i++) {
		fclose(files_open[i]);
		files_open[i] = 0;
	}

	file_now = 0;

	if (as504x_init_done) {
		enc_as504x_deinit(&as504x);
		as504x_init_done = false;
	}

	while (cmds_running) {
		vTaskDelay(1);
	}

	cmds_running = false;
	cmds_state = 0;

	vTaskDelay(pdMS_TO_TICKS(5));
}

void lispif_process_can(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	if (is_ext) {
		if (can_recv_eid_cid < 0 && !event_can_eid_en)  {
			return;
		}
	} else {
		if (can_recv_sid_cid < 0 && !event_can_sid_en)  {
			return;
		}
	}

	lbm_flat_value_t v;
	if (start_flatten_with_gc(&v, 50 + len)) {
		f_cons(&v);

		if ((can_recv_sid_cid < 0 && !is_ext) || (can_recv_eid_cid < 0 && is_ext)) {
			f_sym(&v, is_ext ? sym_event_can_eid : sym_event_can_sid);
			f_cons(&v);
			f_i32(&v, can_id);
			f_lbm_array(&v, len, data8);
		} else {
			f_i32(&v, can_id);
			f_cons(&v);
			f_lbm_array(&v, len, data8);
			f_sym(&v, ENC_SYM_NIL);
		}

		lbm_finish_flatten(&v);

		if (can_recv_sid_cid >= 0 && !is_ext) {
			if (!lbm_unblock_ctx(can_recv_sid_cid, &v)) {
				lbm_free(v.buf);
			}
			can_recv_sid_cid = -1;
		} else if (can_recv_eid_cid >= 0 && is_ext) {
			if (!lbm_unblock_ctx(can_recv_eid_cid, &v)) {
				lbm_free(v.buf);
			}
			can_recv_eid_cid = -1;
		} else {
			if (!lbm_event(&v)) {
				lbm_free(v.buf);
			}
		}
	}
}

void lispif_process_custom_app_data(unsigned char *data, unsigned int len) {
	if (!event_data_rx_en && recv_data_cid < 0) {
		return;
	}

	lbm_flat_value_t v;
	if (start_flatten_with_gc(&v, 30 + len)) {
		if (recv_data_cid < 0) {
			f_cons(&v);
			f_sym(&v, sym_event_data_rx);
		}
		f_lbm_array(&v, len, data);
		lbm_finish_flatten(&v);

		if (recv_data_cid >= 0) {
			if (!lbm_unblock_ctx(recv_data_cid, &v)) {
				lbm_free(v.buf);
			}
			recv_data_cid = -1;
		} else {
			if (!lbm_event(&v)) {
				lbm_free(v.buf);
			}
		}
	}
}

void lispif_process_rmsg(int slot, unsigned char *data, unsigned int len) {
	if (!event_task_running) {
		return;
	}

	xSemaphoreTake(rmsg_mutex, portMAX_DELAY);

	if (slot < 0 || slot >= RMSG_SLOT_NUM || rmsg_slots[slot].cid < 0) {
		xSemaphoreGive(rmsg_mutex);
		return;
	}

	lbm_flat_value_t v;
	if (start_flatten_with_gc(&v, 10 + len)) {
		f_lbm_array(&v, len, data);
		lbm_finish_flatten(&v);

		if (lbm_unblock_ctx(rmsg_slots[slot].cid, &v)) {
			rmsg_slots[slot].cid = -1;
		} else {
			lbm_free(v.buf);
		}
	}

	xSemaphoreGive(rmsg_mutex);
}

char* lispif_print_prefix(void) {
	print_prefix[sizeof(print_prefix) - 1] = 0;
	return print_prefix;
}

char* lispif_fw_name(void) {
	fw_name[sizeof(fw_name) - 1] = 0;
	return fw_name;
}
