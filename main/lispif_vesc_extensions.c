/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se
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

#include "lispif.h"
#include "lispbm.h"
#include "extensions/array_extensions.h"
#include "extensions/string_extensions.h"
#include "extensions/math_extensions.h"
#include "lbm_constants.h"

#include "commands.h"
#include "comm_can.h"
#include "conf_general.h"
#include "mempools.h"
#include "log.h"
#include "buffer.h"
#include "utils.h"

#include <math.h>
#include <ctype.h>
#include <stdarg.h>

static void(*ext_callback)(void) = 0;
static char print_val_buffer[256];

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

static lbm_value ext_get_adc(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

#ifdef HW_HAS_ADC
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
		} else {
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
static lbm_uint sym_event_can_sid;
static lbm_uint sym_event_can_eid;
static lbm_uint sym_event_data_rx;

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

static lbm_value ext_empty(lbm_value *args, lbm_uint argn) {
	(void)args;(void)argn;
	return ENC_SYM_TRUE;
}

void lispif_load_vesc_extensions(void) {
	lbm_add_symbol_const("event-can-sid", &sym_event_can_sid);
	lbm_add_symbol_const("event-can-eid", &sym_event_can_eid);
	lbm_add_symbol_const("event-data-rx", &sym_event_data_rx);

	lbm_add_symbol_const("a01", &sym_res);
	lbm_add_symbol_const("a02", &sym_loop);
	lbm_add_symbol_const("break", &sym_break);
	lbm_add_symbol_const("a03", &sym_brk);
	lbm_add_symbol_const("a04", &sym_rst);

	// Various commands
	lbm_add_extension("print", ext_print);
	lbm_add_extension("get-adc", ext_get_adc);
	lbm_add_extension("systime", ext_systime);
	lbm_add_extension("secs-since", ext_secs_since);
	lbm_add_extension("event-enable", ext_enable_event);
	lbm_add_extension("send-data", ext_send_data);
	lbm_add_extension("import", ext_empty);

	// CAN-comands
	lbm_add_extension("can-scan", ext_can_scan);
	lbm_add_extension("can-send-sid", ext_can_send_sid);
	lbm_add_extension("can-send-eid", ext_can_send_eid);
	lbm_add_extension("can-cmd", ext_can_cmd);

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

	// Extension libraries
	lbm_array_extensions_init();
	lbm_string_extensions_init();
//	lbm_math_extensions_init(); // These make the ESP crash for some reason...

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

	lbm_event_t e = {0};
	e.type = LBM_EVENT_SYM_INT_ARRAY;
	e.sym = is_ext ? sym_event_can_eid : sym_event_can_sid;
	e.i = can_id;
	lbm_event(e, data8, len);
}

void lispif_process_custom_app_data(unsigned char *data, unsigned int len) {
	if (!event_data_rx_en) {
		return;
	}

	lbm_event_t e = {0};
	e.type = LBM_EVENT_SYM_ARRAY;
	e.sym = sym_event_data_rx;
	lbm_event(e, data, len);
}

