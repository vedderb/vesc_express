/*
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com
	Copyright 2023 - 2024 Benjamin Vedder      benjamin@vedder.se

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

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_netif_types.h"
#include "esp_event_base.h"
#include "errno.h"
#include "lwip/api.h"

#include "eval_cps.h"
#include "heap.h"
#include "lbm_defines.h"
#include "lbm_types.h"
#include "lbm_memory.h"
#include "lbm_flat_value.h"
#include "lbm_c_interop.h"
#include "extensions.h"

#include "utils.h"
#include "lispif_events.h"
#include "lbm_vesc_utils.h"
#include "datatypes.h"
#include "commands.h"
#include "comm_wifi.h"
#include "lispif.h"

#define SSID_SIZE SIZEOF_MEMBER(wifi_ap_record_t, ssid)

#define MUTEX_LOCK_TIMEOUT_MS 10

/**
 * Error reasons
 */

static char *error_mode_invalid = "Invalid WIFI mode for this command";
static char *error_thread_waiting =
	"Another thread is currently executing WIFI commands.";
static char *error_wifi_connecting       = "Currently connecting to network.";
static char *error_esp_no_memory         = "ESP ran out of memory Internally.";
static char *error_esp_too_long_ssid     = "Too long ssid, max: 31 chars.";
static char *error_esp_too_long_password = "Too long password, max: 63 chars.";

static lbm_uint symbol_wrong_password = 0;
static lbm_uint symbol_unknown_host   = 0;
static lbm_uint symbol_no_data        = 0;
static lbm_uint symbol_connected      = 0;
static lbm_uint symbol_connecting     = 0;
static lbm_uint symbol_disconnected   = 0;
static lbm_uint symbol_socket_error   = 0;
static lbm_uint symbol_connect_error  = 0;

static volatile bool init_done = false;

static bool register_symbols(void) {
	bool res = true;

	res = res
		&& lbm_add_symbol_const_if_new(
			  "wrong-password", &symbol_wrong_password
		);
	res = res
		&& lbm_add_symbol_const_if_new("unknown-host", &symbol_unknown_host);
	res = res && lbm_add_symbol_const_if_new("no-data", &symbol_no_data);
	res = res && lbm_add_symbol_const_if_new("connected", &symbol_connected);
	res = res && lbm_add_symbol_const_if_new("connecting", &symbol_connecting);
	res = res
		&& lbm_add_symbol_const_if_new("disconnected", &symbol_disconnected);
	res = res
		&& lbm_add_symbol_const_if_new("socket-error", &symbol_socket_error);
	res = res
		&& lbm_add_symbol_const_if_new("connect-error", &symbol_connect_error);
	return res;
}

// For the even listener callback.
typedef enum {
	WAITING_OP_SCAN_AP = 0,
	WAITING_OP_CHANGE_NETWORK,
} waiting_op_t;

static volatile bool is_waiting;
static volatile waiting_op_t waiting_op;
static volatile lbm_cid waiting_cid;

static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static wifi_event_ftm_report_t ftm_report;

/**
 * Checks that the correct WIFI was configured in the custom config, and sets
 * the error reason if it wasn't.
 *
 * Also checks that no other lbm thread is currently executing parts of the WIFI
 * API.
 */
static bool check_mode(bool station_only) {
	if (is_waiting) {
		lbm_set_error_reason(error_thread_waiting);
		return false;
	}

	if (station_only) {
		if (comm_wifi_get_mode() != WIFI_MODE_STATION) {
			lbm_set_error_reason(error_mode_invalid);
			return false;
		}
	} else {
		if (comm_wifi_get_mode() == WIFI_MODE_DISABLED) {
			lbm_set_error_reason(error_mode_invalid);
			return false;
		}
	}

	return true;
}

/**
 * Send lbm wifi disconnect event if it's enabled and wifi is in the correct
 * mode.
 *
 * Does not check if this specific disconnect reason is a type that should be
 * reported!
 *
 * @param reason The disconnect wifi reason code, as found in the
 * wifi_event_sta_disconnected_t struct.
 */
static void handle_wifi_disconnect_event(uint8_t reason, bool from_extension) {
	if (!event_wifi_disconnect_en ||
			comm_wifi_get_mode() != WIFI_MODE_STATION) {
		return;
	}

	// produces ('event_wifi_disconnect reason-code from-extension)
	lbm_flat_value_t flat;
	if (!lbm_start_flatten(&flat, 40)) {
		return;
	}

	f_cons(&flat);                           // +1
	f_sym(&flat, sym_event_wifi_disconnect); // +5/+9

	f_cons(&flat);      // +1
	f_u(&flat, reason); // +5

	f_cons(&flat);                                     // +1
	f_sym(&flat, from_extension ? SYM_TRUE : SYM_NIL); // +5/+9

	f_sym(&flat, SYM_NIL); // +5/+9

	if (!lbm_event(&flat)) {
		STORED_LOGF(
			"failed to send lbm wifi-disconnect event, disconnect_reason: %u", reason
		);
		lbm_free(flat.buf);
	}
}

static void event_listener(
	esp_event_base_t event_base, int32_t event_id, void *event_data
) {
	void return_unboxed(lbm_value value) {
		lbm_unblock_ctx_unboxed(waiting_cid, value);

		is_waiting = false;
	}
	void return_flat(lbm_flat_value_t * value) {
		lbm_unblock_ctx(waiting_cid, value);
		
		is_waiting = false;
	}

	if (event_base == WIFI_EVENT) {
		STORED_LOGF("WIFI event: %d", event_id);
	} else if (event_base == IP_EVENT) {
		STORED_LOGF("IP event: %d", event_id);
	} else {
		STORED_LOGF("Unknown event base %p, id: %d", event_base, event_id);
	}

	if (event_base == WIFI_EVENT) {
		switch (event_id) {
			case WIFI_EVENT_SCAN_DONE: {
				if (is_waiting && waiting_op == WAITING_OP_SCAN_AP) {
					uint16_t len;
					{
						esp_err_t result = esp_wifi_scan_get_ap_num(&len);
						switch (result) {
							case ESP_OK: {
								break;
							}
							case ESP_ERR_INVALID_ARG: {
								// Does this ever happen?
								esp_wifi_clear_ap_list();
								return_unboxed(ENC_SYM_EERROR);
								return;
							}
							case ESP_ERR_WIFI_NOT_INIT:
							case ESP_ERR_WIFI_NOT_STARTED:
							default: {
								esp_wifi_clear_ap_list();
								return_unboxed(ENC_SYM_EERROR);
								return;
							}
						}
					}

					wifi_ap_record_t records[len];
					{
						esp_err_t result =
							esp_wifi_scan_get_ap_records(&len, records);
						switch (result) {
							case ESP_OK: {
								break;
							}
							case ESP_ERR_NO_MEM: {
								lbm_set_error_reason(error_esp_no_memory);
								// TODO: Is this necessary?
								esp_wifi_clear_ap_list();
								return_unboxed(ENC_SYM_FATAL_ERROR);
								return;
							}
							default: {
								// TODO: Is this necessary?
								esp_wifi_clear_ap_list();
								return_unboxed(ENC_SYM_EERROR);
								return;
							}
						}
					}

					lbm_flat_value_t value;
					{
						// +10: to be safe
						size_t size = 9 + 10 + 80 * len;
						for (size_t i = 0; i < len; i++) {
							size += strlen((char *)records[i].ssid);
						}

						if (!lbm_start_flatten(&value, size)) {
							return_unboxed(ENC_SYM_EERROR);
							return;
						}
					}
					/* produces:
					(
						..(ssid rssi channel ftm-responder (mac-addr))
					) */
					for (size_t i = 0; i < len; i++) {
						// 80
						// this one belongs to the outer SYM_NIL.
						f_cons(&value); // +1

						f_cons(&value); // +1
						size_t ssid_len = strlen((char *)records[i].ssid);
						// +5 + ssid_len
						f_lbm_array(&value, ssid_len + 1, records[i].ssid);

						f_cons(&value);               // +1
						f_i(&value, records[i].rssi); // +5

						f_cons(&value);                  // +1
						f_i(&value, records[i].primary); // +5

						f_cons(&value);                  // +1
						f_i(&value, records[i].ftm_responder + 2 * records[i].ftm_initiator); // +5

						f_cons(&value);                   // +1
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[0]); // +5
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[1]); // +5
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[2]); // +5
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[3]); // +5
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[4]); // +5
						f_cons(&value);                   // +1
						f_i(&value, records[i].bssid[5]); // +5
						f_sym(&value, SYM_NIL); // +9

						f_sym(&value, SYM_NIL); // +9
					}
					f_sym(&value, SYM_NIL); // +9

					return_flat(&value);
				}
				break;
			}
			case WIFI_EVENT_STA_DISCONNECTED: {
				wifi_event_sta_disconnected_t *data =
					(wifi_event_sta_disconnected_t *)event_data;

				bool extension_waiting = is_waiting
					&& waiting_op == WAITING_OP_CHANGE_NETWORK;

				bool wifi_is_reconnecting = comm_wifi_is_connecting()
					|| comm_wifi_is_connected();

				if (!wifi_is_reconnecting) {
					handle_wifi_disconnect_event(
						data->reason, extension_waiting
					);
				}

				if (extension_waiting) {
					// From ESP WIFI docs:
					// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-reason-code-related-to-wrong-password
					bool is_wrong_password = data->reason
							== WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT
						|| data->reason == WIFI_REASON_NO_AP_FOUND
						|| data->reason == WIFI_REASON_HANDSHAKE_TIMEOUT;

					// These were just kind of found through testing,
					// and I'm not entirely sure when or why they happen
					// (except for the first one). These mean that we're
					// not sure why the connection failed, and that we
					// should wait for the next reconnect attempt and
					// hope for a different outcome.
					bool is_undetermined_disconnect =
						// This indicates the event was caused by the
						// network change and is normal.
						data->reason == WIFI_REASON_ASSOC_LEAVE
						// AUTH_EXPIRE can unfortunately occur both when
						// connecting to a network for the first time in a while
						// (like maybe a few hours?) with wrong *or* correct
						// credentials, so we need to try to connect a second
						// time to get a usefull answer.
						|| data->reason == WIFI_REASON_AUTH_EXPIRE;

					if (is_wrong_password) {
						return_unboxed(ENC_SYM(symbol_wrong_password));
					} else if (!is_undetermined_disconnect) {
						return_unboxed(ENC_SYM_NIL);
					}
				}
				break;
			}
		}
	} else if (event_base == IP_EVENT) {
		switch (event_id) {
			case IP_EVENT_STA_GOT_IP: {
				if (is_waiting && waiting_op == WAITING_OP_CHANGE_NETWORK) {
					return_unboxed(ENC_SYM_TRUE);
				}
				break;
			}
		}
	}

	if (event_id == WIFI_EVENT_FTM_REPORT) {
		wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *) event_data;

		memcpy(&ftm_report, event, sizeof(ftm_report));
		xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
	}
}

/**
 * signature: (wifi-scan-networks [scan-time:number]
 * [channel:number] [show-hidden:bool]) -> ssids where ssids = list
 * of network-tuple network-tuple = (ssid:str rssi:number
 * channel:number)
 *
 * Perform a passive scan of all nearby visible networks and return
 * a list of the results.
 *
 * @attention This function currently blocks for the entire duration
 * of the scan in a non-concurrent way unfortunately.
 *
 * @param scan_time [optional] How long to scan each channel for in
 * seconds. If an all-channel scan is specified (setting channel to
 * 0), you can multiply this number by 14 (the amount of channels)
 * to get an approximate total time of the scan. (Default: 0.12)
 * @param channel [optional] Which 2.4GHz (I think...) wifi channel
 * to scan for. If 0 is specificed, all channels 1-14 (inclusive)
 * will be scanned. (Default: 0)
 * @param show_hidden [optional] Include hidden networks in scan
 * results. (Default: nil)
 * @return List of 3 element tuples corresponding to the found
 * networks.
 */
static lbm_value ext_wifi_scan_networks(lbm_value *args, lbm_uint argn) {
	uint32_t scan_time = 120;
	if (argn >= 1) {
		scan_time = (uint32_t)(lbm_dec_as_float(args[0]) * 1000);
	}

	uint8_t channel = 0;
	if (argn >= 2) {
		channel = lbm_dec_as_u32(args[1]);
	}

	bool show_hidden = false;
	if (argn >= 3) {
		show_hidden = lbm_dec_bool(args[2]);
	}

	// See documentation for config parameters:
	// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#scan-configuration
	wifi_scan_config_t config = {
		.bssid       = NULL,
		.ssid        = NULL,
		.channel     = channel,
		.scan_type   = WIFI_SCAN_TYPE_PASSIVE,
		.show_hidden = show_hidden,
		.scan_time   = {.active = {scan_time, scan_time}, .passive = scan_time},
	};

	esp_err_t result = esp_wifi_scan_start(&config, false);
	switch (result) {
		case ESP_OK: {
			break;
		}
		case ESP_ERR_WIFI_NOT_STARTED: {
			// Should not be possible.
			return ENC_SYM_EERROR;
		}
		case ESP_ERR_WIFI_STATE: {
			lbm_set_error_reason(error_wifi_connecting);
			return ENC_SYM_EERROR;
		}
		case ESP_ERR_WIFI_TIMEOUT:
		default: {
			return ENC_SYM_EERROR;
		}
	}

	waiting_cid = lbm_get_current_cid();
	lbm_block_ctx_from_extension();
	waiting_op = WAITING_OP_SCAN_AP;
	is_waiting = true;

	return ENC_SYM_NIL;
}

/**
 * signature: (wifi-connect ssid:string password:string|nil) -> bool
 *
 * Connect to the specified wifi network.
 *
 * @param ssid The wifi network's name. Must not be longer than 31
 * characters (excluding the null byte).
 * @param password The network's password. Pass nil if the network
 * doesn't have a password. Must not be longer than 63 characters
 * (excluding the null byte).
 * @return True if the connection was successfull (the connection is
 * then ready for opening tcp sockets at this point). Otherwise
 * 'wrong_password is returned if the password or ssid was incorrect
 * (it's not very reliable though). Finally, nil is returned if the
 * connection failed for some other reason.
 * @throw Throws an eval_error if wifi is not set to station mode in
 * the VESC Express config or if the ssid or password are too long.
 */
static lbm_value ext_wifi_connect(lbm_value *args, lbm_uint argn) {
	if (!check_mode(true)) {
		return ENC_SYM_EERROR;
	}

	LBM_CHECK_ARGN(2);

	if (!lbm_is_array_r(args[0])
		|| !(lbm_is_array_r(args[1]) || lbm_is_symbol_nil(args[1]))) {
		return ENC_SYM_TERROR;
	}

	const char *ssid = lbm_dec_str(args[0]);
	if (!ssid) {
		// Should be impossible.
		return ENC_SYM_FATAL_ERROR;
	}
	if (strlen(ssid) >= 31) {
		lbm_set_error_reason(error_esp_too_long_ssid);
		return ENC_SYM_EERROR;
	}

	// A nil value will return a null pointer here.
	const char *password = lbm_dec_str(args[1]);
	if (!password) {
		// Should be impossible.
		return ENC_SYM_FATAL_ERROR;
	}
	if (strlen(password) >= 63) {
		lbm_set_error_reason(error_esp_too_long_password);
		return ENC_SYM_EERROR;
	}

	waiting_cid = lbm_get_current_cid();
	waiting_op  = WAITING_OP_CHANGE_NETWORK;
	is_waiting  = true;

	bool result = comm_wifi_change_network(ssid, password);
	if (!result) {
		return ENC_SYM_NIL;
	}

	lbm_block_ctx_from_extension();
	return ENC_SYM_NIL;
}

/**
 * signature: (wifi-disconnect)
 *
 * Disconnect from any currently connected WIFI networks.
 */
static lbm_value ext_wifi_disconnect(lbm_value *args, lbm_uint argn) {
	if (!check_mode(true)) {
		return ENC_SYM_EERROR;
	}
	comm_wifi_disconnect_network();

	return ENC_SYM_TRUE;
}

/**
 * signature: (wifi-status) -> status
 * where
 *   status = 'connected|'connecting|'disconnected
 *
 * Check the current WIFI connection status.
 */
static lbm_value ext_wifi_status(lbm_value *args, lbm_uint argn) {
	if (!check_mode(true)) {
		return ENC_SYM_EERROR;
	}

	if (comm_wifi_is_connecting()) {
		return ENC_SYM(symbol_connecting);
	} else if (comm_wifi_is_connected()) {
		return ENC_SYM(symbol_connected);
	} else {
		return ENC_SYM(symbol_disconnected);
	}
}

/**
 * signature: (wifi-auto-reconnect [should-reconnect:bool]) -> bool
 *
 * Set if the internal event handler should automatically attempt to reconnect
 * to the current wifi network on disconnects.
 *
 * The wifi module already doesn't reconnect if it's known that the disconnect
 * was due to incorrect credentials.
 *
 * A lbm event is always sent whenever the wifi disconnects and the wifi module
 * doesn't try to reconnect. It is then the LBM codes responsibility to take
 * action and reconnect.
 *
 * @param should_reconnect [optional] If the wifi module should attempt to
 * reconnect on unknown disconnects. Leave out to instead query the current
 * value, without modifying it.
 * @return The previous setting, or the current if should-reconnect wasn't
 * passed.
 */
static lbm_value ext_wifi_auto_reconnect(lbm_value *args, lbm_uint argn) {
	if (!check_mode(true)) {
		return ENC_SYM_EERROR;
	}

	bool current_value = comm_wifi_get_auto_reconnect();

	if (argn == 0) {
		return lbm_enc_bool(current_value);
	}

	if (!lbm_is_bool(args[0])) {
		return ENC_SYM_TERROR;
	}

	bool should_reconnect = lbm_dec_bool(args[0]);

	// we can ignore the return value since we already checked the mode.
	comm_wifi_set_auto_reconnect(should_reconnect);

	return lbm_enc_bool(current_value);
}

typedef struct {
	lbm_cid id;
	wifi_ftm_initiator_cfg_t cfg;
	bool print;
} ftm_args;

static void ftm_task(void *arg) {
	ftm_args *a = (ftm_args*)arg;
	int restart_cnt = lispif_get_restart_cnt();

	lbm_value res = ENC_SYM_NIL;

	if (esp_wifi_ftm_initiate_session(&a->cfg) != ESP_OK) {
		if (a->print) {
			commands_printf_lisp("Failed to start FTM session");
		}
		goto end;
	}

	uint32_t wait_time_ms = 500;

	EventBits_t bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT,
			pdTRUE, pdFALSE, wait_time_ms / portTICK_PERIOD_MS);

	if (bits & FTM_REPORT_BIT) {
		esp_wifi_ftm_get_report(NULL, 0);

		if (ftm_report.status == FTM_STATUS_SUCCESS) {
			res =  lbm_enc_i(ftm_report.dist_est);
		} else if (ftm_report.status == FTM_STATUS_UNSUPPORTED) {
			if (a->print) {
				commands_printf_lisp("FTM not supported by peer");
			}
		} else if (ftm_report.status == FTM_STATUS_CONF_REJECTED) {
			if (a->print) {
				commands_printf_lisp("FTM configuration rejected by peer");
			}
		} else if (ftm_report.status == FTM_STATUS_NO_RESPONSE) {
			if (a->print) {
				commands_printf_lisp("FTM no response");
			}
		} else {
			if (a->print) {
				commands_printf_lisp("FTM failed");
			}
		}
	} else {
		esp_wifi_ftm_end_session();
		if (a->print) {
			commands_printf_lisp("FTM timed out");
		}
	}

	end:

	if (restart_cnt == lispif_get_restart_cnt()) {
		vTaskDelay(1);
		lbm_unblock_ctx_unboxed(a->id, res);
	}

	lbm_free(a);

	vTaskDelete(NULL);
}

static lbm_value ext_wifi_ftm_measure(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && argn != 3) {
		lbm_set_error_reason((char*)lbm_error_str_num_args);
		return ENC_SYM_TERROR;
	}

	if (!lbm_is_number(args[1])) {
		lbm_set_error_reason((char*)lbm_error_str_no_number);
		return ENC_SYM_TERROR;
	}

	ftm_args *a = lbm_malloc(sizeof(ftm_args));

	if (!a) {
		return ENC_SYM_MERROR;
	}

	memset(a, 0, sizeof(ftm_args));

	if (argn >= 3) {
		if (!lbm_is_symbol_nil(args[2])) {
			a->print = true;
		}
	}

	int ind = 0;

	lbm_value curr = args[0];
	while (lbm_is_cons(curr)) {
		lbm_value  arg = lbm_car(curr);

		if (lbm_is_number(arg)) {
			a->cfg.resp_mac[ind++] = lbm_dec_as_u32(arg);
		} else {
			lbm_free(a);
			return ENC_SYM_TERROR;
		}

		if (ind == sizeof(a->cfg.resp_mac)) {
			break;
		}

		curr = lbm_cdr(curr);
	}


	a->id = lbm_get_current_cid();
	a->cfg.use_get_report_api = true;
	a->cfg.channel = lbm_dec_as_i32(args[1]);
	a->cfg.frm_count = 8;
	a->cfg.burst_period = 2;

	lbm_block_ctx_from_extension();

	xTaskCreatePinnedToCore(ftm_task, "FTM Measure", 2048, a, 7, NULL, tskNO_AFFINITY);

	return ENC_SYM_NIL;
}

#define CUSTOM_SOCKET_COUNT 5
static int custom_sockets[CUSTOM_SOCKET_COUNT];
static int custom_socket_now = 0;

static bool custom_socket_valid(int socket) {
	if (socket < 0) {
		return false;
	}

	for (int i = 0;i < CUSTOM_SOCKET_COUNT;i++) {
		if (custom_sockets[i] == socket) {
			return true;
		}
	}

	return false;
}

/**
 * signature: (tcp-connect dest:str port:number) -> number|nil|error
 * where
 *   error = 'unknown-host
 *
 * Open a new tcp socket connected to the specificed destination
 * hostname/IP address.
 *
 * @param dest The hostname/IPv4 address to connect to as a string.
 * IPv4 addresses are written using the normal IP dot notation (ex:
 * "127.0.0.1").
 * @param port The port of the host to connect to. Will be casted to
 * a a 16-bit unsigned integer.
 * @return todo
 */
static lbm_value ext_tcp_connect(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 2)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_array_r(args[0]) || !lbm_is_number(args[1])) {
		return ENC_SYM_TERROR;
	}

	const char *host = lbm_dec_str(args[0]);
	if (!host) {
		// should be impossible
		return ENC_SYM_FATAL_ERROR;
	}

	const uint16_t port = lbm_dec_as_u32(args[1]);

	ip_addr_t ip_addr;
	{
		err_t result = netconn_gethostbyname(host, &ip_addr);
		if (result != ERR_OK) {
			STORED_LOGF("netconn_gethostbyname failed, result: %d", result);
			return ENC_SYM(symbol_unknown_host);
		}
	}

	struct sockaddr_in addr = create_sockaddr_in(ip_addr, port);

	if (custom_socket_now >= CUSTOM_SOCKET_COUNT) {
		lbm_set_error_reason("Too many sockets open.");
		return ENC_SYM_EERROR;
	}

	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

	if (sock < 0) {
		char *errstr        = strerror(errno);
		lbm_value errstrval = ENC_SYM_NIL;
		if (lbm_lift_array(&errstrval, errstr, strlen(errstr) + 1) == 0) {
			return errstrval;
		}

		lbm_value errval = ENC_SYM_NIL;
		errval           = lbm_cons(errstrval, errval);
		errval           = lbm_cons(ENC_SYM(symbol_socket_error), errval);
		return errval;
	}

	{
		int result = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
		if (result != 0) {

			char *errstr        = strerror(errno);
			lbm_value errstrval = ENC_SYM_NIL;
			if (lbm_lift_array(&errstrval, errstr, strlen(errstr) + 1) == 0) {
				return errstrval;
			}

			shutdown(sock, 0);
			close(sock);
			lbm_value errval = ENC_SYM_NIL;
			errval           = lbm_cons(errstrval, errval);
			errval           = lbm_cons(ENC_SYM(symbol_connect_error), errval);
			return errval;
		}
	}


	custom_sockets[custom_socket_now++] = sock;

	// TODO: Add keep alive configuration options.
	int keep_alive    = true;
	int keep_idle     = 5;
	int keep_interval = 5;
	int keep_count    = 3;
	int no_delay      = true;
	// TODO: error checking?
	setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keep_alive, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keep_interval, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keep_count, sizeof(int));
	setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(int));

	return lbm_enc_i(sock);
}

/**
 * signature: (tcp-close socket:number) -> bool
 *
 * Close a tcp connection created by tcp-connect.
 *
 * Note that you still need to call this when the server has already
 * disconnected and the socket is unusable.
 *
 * @return true on success, or nil either if the provided socket
 * didn't exist/wasn't valid or if some other error occurred while
 * closing the socket
 * (@todo: be more precise).
 */
static lbm_value ext_tcp_close(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 1)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	bool socket_found = false;
	int socket_ind = 0;
	for (int i = 0;i < CUSTOM_SOCKET_COUNT;i++) {
		if (sock == custom_sockets[i]) {
			socket_found = true;
			socket_ind = i;
			break;
		}
	}

	if (!socket_found) {
		return ENC_SYM_NIL;
	}

	for (int i = socket_ind;i < custom_socket_now;i++) {
		custom_sockets[i] = custom_sockets[i + 1];
	}

	custom_socket_now--;

	shutdown(sock, 0);
	close(sock);

	return ENC_SYM_TRUE;
}

/**
 * signature: (tcp-status socket:number) -> status|nil
 * where
 *   status = 'connected|'disconnected
 *
 * Query the connection status of a tcp socket.
 *
 * @return One of the following status symbols:
 * - 'connected: The socket is currently connected to the remote,
 * and it's possible to send and receive data.
 * - 'disconnected: The connection has been closed by the remote,
 * and it's not possible to send any more data. There might still be
 * unread received data left though (TODO: check that this is true).
 * - nil: The provided socket either didn't exist, or was already
 * closed using tcp-close (or potentially also by some other
 * internal process, that shouldn't happen).
 */
static lbm_value ext_tcp_status(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 1)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	if (!custom_socket_valid(sock)) {
		STORED_LOGF("socket %d did not exist in registry", sock);
		return ENC_SYM_NIL;
	}

	// Setting array length to 1 probably isn't the most elegant way
	// to do it...
	bool connected;
	char buffer[1];
	ssize_t len = recv(sock, buffer, 1, MSG_DONTWAIT | MSG_PEEK);
	if (len != -1) {
		connected = len != 0;
	} else {
		switch (errno) {
			case EWOULDBLOCK: {
				connected = true;
				break;
			}
			// These cases have been determined by testing and good
			// ol' guessing.
			case ECONNRESET:
			case ECONNABORTED:
			case ENOTCONN: {
				connected = false;
				break;
			}
			case ENOTSOCK:
			default: {
				return ENC_SYM_NIL;
			}
		}
	}

	if (connected) {
		return ENC_SYM(symbol_connected);
	} else {
		return ENC_SYM(symbol_disconnected);
	}
}

/**
 * signature: (tcp-send socket:number data:byte-array) -> bool
 *
 * @todo: Document this
 */
static lbm_value ext_tcp_send(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 2)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_array_r(args[1])) {
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	const lbm_array_header_t *array = lbm_dec_array_header(args[1]);
	size_t size                     = (size_t)array->size;
	const char *data                = (char *)array->data;
	if (!array || !array->data) {
		// Should be impossible.
		return ENC_SYM_FATAL_ERROR;
	}

	ssize_t len = send(sock, data, size, 0);
	if (len == -1) {
		switch (errno) {
			// Trying to send remote has disconnected seems to
			// generate a ECONNABORTED the first time, then ENOTCONN
			// on subsequent attempts for whatever reason. I think
			// this should be fine either way.
			case ECONNABORTED:
			// Not sure if ECONNRESET is necessary, it's just to be
			// safe.
			case ECONNRESET:
			case ENOTCONN: {
				return ENC_SYM(symbol_disconnected);
			}
			default: {
				return ENC_SYM_NIL;
			}
		}
	}

	return ENC_SYM_TRUE;
}

typedef struct {
	lbm_cid return_cid;
	int socket;
	lbm_value buffer;
	bool as_str;
	bool return_on_disconnect;
	char terminator;
	float timeout;
} recv_task_state;

static void recv_task(void *arg) {
	recv_task_state *s = (recv_task_state*)arg;
	lbm_array_header_t *buffer = (lbm_array_header_t*)lbm_car(s->buffer);
	lbm_uint recv_size = buffer->size;
	if (s->as_str) {
		recv_size--;
	}

	int start = xTaskGetTickCount();


	for (;;) {
		ssize_t len = recv(s->socket, (char*)(buffer->data), recv_size, MSG_DONTWAIT);

		if (len < 0) {
			switch (errno) {
			case EWOULDBLOCK: {
				vTaskDelay(1);
				continue;
			}
			case ECONNRESET:
			case ECONNABORTED:
			case ENOTCONN: {
				lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM(symbol_disconnected));
				goto recv_cleanup;
			}
			default: {
				// an error has occurred
				lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM_NIL);
				goto recv_cleanup;
			}
			}
		} else if (len == 0) {
			lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM(symbol_disconnected));
			goto recv_cleanup;
		} else {
			size_t result_size = len;
			if (s->as_str) {
				result_size++;
				((char*)buffer->data)[len] = '\0';
			}

			lbm_array_shrink(s->buffer, result_size);
			lbm_unblock_ctx_r(s->return_cid);

			goto recv_cleanup;
		}

		if (UTILS_AGE_S(start) > s->timeout) {
			STORED_LOGF("timed out after %d seconds", (double)UTILS_AGE_S(start));
			lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM(symbol_no_data));
			break;
		}
	}

	recv_cleanup:

	lbm_free(s);
	vTaskDelete(NULL);
}

// Maybe this can be merged with recv_task?
static void recv_to_char_task(void *arg) {
	recv_task_state *s = (recv_task_state *)arg;
	lbm_array_header_t *buffer = (lbm_array_header_t*)lbm_car(s->buffer);

	size_t total_len = 0;
	size_t result_size = 0;
	lbm_uint recv_size = buffer->size;
	if (s->as_str) {
		recv_size--;
	}

	int start = xTaskGetTickCount();

	while (true) {
		if (recv_size == 0) {
			break;
		}

		uint8_t byte;
		ssize_t len = recv(s->socket, &byte, 1, MSG_DONTWAIT);

		if (len < 0) {
			switch (errno) {
				case EWOULDBLOCK: {
					vTaskDelay(1);
					continue;
				}
				case ECONNRESET:
				case ECONNABORTED:
				case ENOTCONN: {
					if (total_len == 0 || !s->return_on_disconnect) {
						lbm_unblock_ctx_unboxed(
							s->return_cid, ENC_SYM(symbol_disconnected)
						);
						goto recv_cleanup;
					} else {
						goto return_buffer;
					}
				}
				default: {
					// an error has occurred
					lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM_NIL);
					goto recv_cleanup;
				}
			}
		} else if (len == 0) {
			if (total_len == 0 || !s->return_on_disconnect) {
				lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM(symbol_disconnected));
				goto recv_cleanup;
			}
			break;
		} else {
			((char*)buffer->data)[total_len] = byte;
			total_len++;

			if (byte == s->terminator || total_len >= recv_size) {
				break;
			}
		}

		if (UTILS_AGE_S(start) > s->timeout) {
			STORED_LOGF(
				"timed out after %d seconds", (double)UTILS_AGE_S(start)
			);
			if (total_len == 0) {
				lbm_unblock_ctx_unboxed(s->return_cid, ENC_SYM(symbol_no_data));
				goto recv_cleanup;
			} else {
				break;
			}
		}
	}

return_buffer:

	result_size = total_len;
	if (s->as_str) {
		result_size++;
		((char*)buffer->data)[total_len] = '\0';
	}

	lbm_array_shrink(s->buffer, result_size);
	lbm_unblock_ctx_r(s->return_cid);

recv_cleanup:

	lbm_free(s);
	vTaskDelete(NULL);
}

/**
 * signature: (tcp-recv socket:number max-len:number
 * [timeout:number|nil] [as-str:bool]) -> byte-array|nil
 *
 * @param socket The socket to receive data over. Should have been
 * created using tcp-connect.
 * @param max_len The amount of bytes to receive at most.
 * @param timeout [optional] The amount of seconds to wait for data
 * *at least*. Pass nil to only receive data that is immediately
 * available. (Default: 1.0)
 * @param as_str [optional] Return the received data as a string.
 * If true is passed, an additional terminating null byte is
 * appended to the received buffer, to let it be interpreted as a
 * string. (The maximum length of the returned buffer is then
 * max_len + 1 to account for the terminating null byte.) If false
 * is passed, the literal raw binary data received is returned.
 * (Default: true)
 * @return The received data, 'no-data if no data was received
 * (either because there were none available this instant, or
 * because the timeout was reached). 'disconnected is returned if
 * the remote has closed the connection and there is no more data to
 * receive. On other errors nil is returned, such as when an invalid
 * socket was provided, or if another unknown (at the time of
 * writing this) network error occurred.
 */
static lbm_value ext_tcp_recv(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn_range(argn, 2, 4)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_number(args[1])) {
		return ENC_SYM_TERROR;
	}

	int sock       = lbm_dec_as_i32(args[0]);
	size_t max_len = lbm_dec_as_u32(args[1]);

	bool should_wait   = true;
	float timeout_secs = 1.0;
	if (argn >= 3) {
		if (!lbm_is_number(args[2]) && !lbm_is_symbol_nil(args[2])) {
			return ENC_SYM_TERROR;
		}
		should_wait = !lbm_is_symbol_nil(args[2]);
		if (should_wait) {
			timeout_secs = lbm_dec_as_float(args[2]);
		}
	}

	bool as_str = true;
	if (argn >= 4) {
		if (!lbm_is_bool(args[3])) {
			return ENC_SYM_TERROR;
		}

		as_str = lbm_dec_bool(args[3]);
	}

	lbm_value result;
	size_t size = max_len;
	if (as_str) {
		size++;
	}
	if (!lbm_create_array(&result, size)) {
		return ENC_SYM_MERROR;
	}

	if (should_wait) {
		recv_task_state *s = lbm_malloc(sizeof(recv_task_state));

		if (s) {
			lbm_block_ctx_from_extension();

			s->return_cid = lbm_get_current_cid();
			s->buffer = result;
			s->timeout = timeout_secs;
			s->socket = sock;
			s->as_str = as_str;

			xTaskCreatePinnedToCore(
				recv_task, "lbm_sockets", 1024, s, 3, NULL, tskNO_AFFINITY
			);

			return result;
		} else {
			return ENC_SYM_MERROR;
		}
	} else {
		char *buffer = lbm_dec_array_data(result);
		ssize_t len = recv(sock, buffer, max_len, MSG_DONTWAIT);

		if (len == -1) {
			switch (errno) {
				case EWOULDBLOCK: {
					return ENC_SYM(symbol_no_data);
				}
				case ECONNRESET:
				case ECONNABORTED:
				case ENOTCONN: {
					return ENC_SYM(symbol_disconnected);
				}
				default: {
					return ENC_SYM_NIL;
				}
			}
		}

		if (len == 0) {
			// Receiving 0 bytes seems to happen right before
			// getting a ENOTCONN error, which means that the remote
			// has closed the connection. (You might also get 0
			// bytes if the local code has called shutdown somewhere
			// I think, but since these lbm APIs don't do that, we
			// can ignore that case.)
			return ENC_SYM(symbol_disconnected);
		} else {
			size = len;
			if (as_str) {
				size++;
				buffer[len] = '\0';
			}

			lbm_array_shrink(result, size);
		}

		return result;
	}
}

/**
 * signature: (tcp-recv-to-char socket:number max-len:number terminator:char
 * [timeout:number] [as-str:bool] [return-on-disconnect:bool]) -> byte-array|nil
 *
 * Receive a string until the specified character is encountered
 *
 * @param socket The socket to receive data over. Should have been
 * created using tcp-connect.
 * @param max_len The amount of bytes to receive at most. The terminator is
 * included in this length.
 * @param terminator The character to recv until. This character is included in
 * the returned byte array. Will be interpreted as an 8-bit char.
 * @param timeout [optional] The amount of seconds to wait for data * *at
 * least*. (Default: 1.0)
 * @param as_str [optional] Return the received data as a string.
 * If true is passed, an additional terminating null byte is appended to the
 * received buffer, to let it be interpreted as a string. (The maximum length of
 * the returned buffer is then max_len + 1 to account for the terminating null
 * byte.) If false is passed, the literal raw binary data received is returned.
 * (Default: true)
 * @param return_on_disconnect [optional] When the remote disconnects mid
 * receive, return the data that has been received up until that point. If
 * false, the symbol 'disconnected is always returned on disconnects. If no
 * data had been received before the disconnect, the symbol 'disconnected is
 * returned either way. (Default: false)
 * @return The received data including the terminator, 'no-data if no data was
 * received (either because there were none available this instant, or because
 * the timeout was reached). 'disconnected is returned if the remote has closed
 * the connection and there is no more data to receive (see
 * return_on_disconnect). On other errors nil is returned, such as when an
 * invalid socket was provided, or if another unknown (at the time of writing
 * this) network error occurred.
 */
static lbm_value ext_tcp_recv_to_char(lbm_value *args, lbm_uint argn) {
	if (!check_mode(false)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn_range(argn, 3, 6)) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_number(args[1])
		|| !lbm_is_number(args[2])) {
		return ENC_SYM_TERROR;
	}
	
	int sock       = lbm_dec_as_i32(args[0]);
	size_t max_len = lbm_dec_as_u32(args[1]);
	char terminator = lbm_dec_as_char(args[2]);

	float timeout_secs = 1.0;
	if (argn >= 4) {
		if (!lbm_is_number(args[3])) {
			return ENC_SYM_TERROR;
		}
		timeout_secs = lbm_dec_as_float(args[3]);
	}
	
	bool as_str = true;
	if (argn >= 5) {
		if (!lbm_is_bool(args[4])) {
			return ENC_SYM_TERROR;
		}
		as_str = lbm_dec_bool(args[4]);
	}
	
	bool return_on_disconnect = false;
	if (argn >= 6) {
		if (!lbm_is_bool(args[5])) {
			return ENC_SYM_TERROR;
		}
		return_on_disconnect = lbm_dec_bool(args[5]);
	}
	
	lbm_value result;
	size_t size = max_len;
	if (as_str) {
		size++;
	}
	if (!lbm_create_array(&result, size)) {
		return ENC_SYM_MERROR;
	}

	recv_task_state *s = lbm_malloc(sizeof(recv_task_state));
	
	if (s) {
		lbm_block_ctx_from_extension();

		s->return_cid = lbm_get_current_cid();
		s->buffer = result;
		s->timeout = timeout_secs;
		s->socket = sock;
		s->as_str = as_str;
		s->return_on_disconnect = return_on_disconnect;
		s->terminator = terminator;

		xTaskCreatePinnedToCore(
			recv_to_char_task, "lbm_sockets", 1024, s, 3, NULL, tskNO_AFFINITY
		);

		return result;
	} else {
		return ENC_SYM_MERROR;
	}
}

void lispif_load_wifi_extensions(void) {
	if (!init_done) {
		comm_wifi_set_event_listener(event_listener);
		s_ftm_event_group = xEventGroupCreate();

		for (int i = 0;i < CUSTOM_SOCKET_COUNT;i++) {
			custom_sockets[i] = -1;
		}

		init_done = true;
	} else {
		for (int i = 0;i < CUSTOM_SOCKET_COUNT;i++) {
			if (custom_sockets[i] >= 0) {
				shutdown(custom_sockets[i], 0);
				close(custom_sockets[i]);
			}

			custom_sockets[i] = -1;
		}
	}

	custom_socket_now = 0;

	register_symbols();

	lbm_add_extension("wifi-scan-networks", ext_wifi_scan_networks);
	lbm_add_extension("wifi-connect", ext_wifi_connect);
	lbm_add_extension("wifi-disconnect", ext_wifi_disconnect);
	lbm_add_extension("wifi-status", ext_wifi_status);
	lbm_add_extension("wifi-auto-reconnect", ext_wifi_auto_reconnect);
	lbm_add_extension("wifi-ftm-measure", ext_wifi_ftm_measure);
	lbm_add_extension("tcp-connect", ext_tcp_connect);
	lbm_add_extension("tcp-close", ext_tcp_close);
	lbm_add_extension("tcp-status", ext_tcp_status);
	lbm_add_extension("tcp-send", ext_tcp_send);
	lbm_add_extension("tcp-recv", ext_tcp_recv);
	lbm_add_extension("tcp-recv-to-char", ext_tcp_recv_to_char);
}
