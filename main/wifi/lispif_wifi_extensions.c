/*
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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

#include <stdbool.h>

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
#include "lbm_c_interop.h"
#include "extensions.h"

#include "utils.h"
#include "lispif_events.h"
#include "lbm_vesc_utils.h"
#include "datatypes.h"
#include "commands.h"
#include "comm_wifi.h"

#define SSID_SIZE SIZEOF_MEMBER(wifi_ap_record_t, ssid)

#define MUTEX_LOCK_TIMEOUT_MS 10

/**
 * Error reasons
 */

static char *error_mode_invalid = "WIFI not in Station mode.";
static char *error_thread_waiting =
	"Another thread is currently executing WIFI commands.";
static char *error_socket_waiting =
	"Another thread is currently using the TCP API.";
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

	return res;
}

// For the even listener callback.
typedef enum {
	WAITING_OP_CHANGE_NETWORK = 0,
} waiting_op_t;

static volatile _Atomic(bool) is_waiting;
static volatile _Atomic(waiting_op_t) waiting_op;
static volatile _Atomic(lbm_cid) waiting_cid;

// For the socket operations thread. These are set to indicate that an
// extensions wants to the thread to perform some operation.
typedef enum {
	SOCKET_OP_RECV = 0,
	// TASK_OP_SEND,
} socket_op_t;

static SemaphoreHandle_t socket_mutex;
static volatile _Atomic(bool) socket_is_waiting = false;
static volatile _Atomic(socket_op_t) socket_op;
static volatile _Atomic(lbm_cid) socket_waiting_cid;
// operation parameters
static volatile _Atomic(size_t) socket_param_max_len;
static volatile _Atomic(float) socket_param_timeout_secs;
static volatile _Atomic(int) socket_param_sock;
static volatile _Atomic(bool) socket_param_as_str;

/**
 * Checks that the correct WIFI was configured in the custom config, and sets
 * the error reason if it wasn't.
 *
 * Also checks that no other lbm thread is currently executing parts of the WIFI
 * API.
 */
static bool check_mode() {
	if (is_waiting) {
		lbm_set_error_reason(error_thread_waiting);
		return false;
	}
	if (comm_wifi_get_mode() != WIFI_MODE_STATION) {
		lbm_set_error_reason(error_mode_invalid);
		return false;
	}
	return true;
}

/**
 * Obtains the socket task lock, preventing other threads from performing socket
 * operations until this one is completed. (And consequently, preventing this
 * thread from continuing if another operation is being executed in the separate
 * socket task thread.)
 *
 * This should be called at the start of any extension that does any operations
 * on sockets.
 *
 * Make sure that free_socket_task is called once this operation is done, (which
 * might be from another thread)!
 */
static bool lock_socket_task() {
	if (!xSemaphoreTake(
			socket_mutex, MUTEX_LOCK_TIMEOUT_MS / portTICK_PERIOD_MS
		)) {
		lbm_set_error_reason(error_socket_waiting);
		return false;
	}

	return true;
}

static void free_socket_task() {
	// TODO: Can we ignore the return value?
	int result = xSemaphoreGive(socket_mutex);
	(void)result;
}

/**
 * Helper function for ext_wifi_scan_networks.
 *
 * @param count The length of the records array.
 * @param list A ready 2-dimensional lbm list created by
 * `lbm_allocate_empty_list_grid`, with a width of 3 and a height that fits
 * `count` entries. **The validity of list is not checked!** (TODO: or is it?)
 * @param ssid_buffers An array of pre-allocated lbm byte buffers at least
 * SSID_SIZE bytes in size, which will be used when created lbm strings for the
 * SSID entries. **The validity of this these array values is not checked!**
 * @param records An array of network records, whose values will be written
 * to the given lbm list.
 */
bool load_lbm_list_with_ap_records(
	uint16_t count, lbm_value list, lbm_value ssid_buffers[count],
	const wifi_ap_record_t records[count]
) {
	lbm_value current = list;
	for (uint16_t i = 0; i < count; i++) {
		if (!lbm_is_cons(current)) {
			STORED_LOGF("ran out of cons cells in outer list");
			return false;
		}

		lbm_value ssid_str = ssid_buffers[i];
		uint8_t *data      = lbm_heap_array_get_data(ssid_str);
		if (data == NULL) {
			STORED_LOGF(
				"invalid ssid_buffers[%u] data pointer %p, "
				"lbm_is_array_r(ssid_str): %u, lbm_is_array_rw(ssid_str): %u, "
				"type_of: 0x%x",
				i, data, lbm_is_array_r(ssid_str), lbm_is_array_rw(ssid_str),
				lbm_type_of(ssid_str)
			);
			return false;
		}
		memcpy(data, records[i].ssid, SSID_SIZE);
		lbm_value signal_strength = lbm_enc_i(records[i].rssi);
		lbm_value primary_channel = lbm_enc_u(records[i].primary);

		lbm_value entry = lbm_car(current);
		lbm_set_car(entry, ssid_str);
		entry = lbm_cdr(entry);
		lbm_set_car(entry, signal_strength);
		entry = lbm_cdr(entry);
		lbm_set_car(entry, primary_channel);

		current = lbm_cdr(current);
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
	if (!event_wifi_disconnect_en
		|| comm_wifi_get_mode() != WIFI_MODE_STATION) {
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
			"failed to send lbm wifi-disconnect event, reason: %u", reason
		);
		lbm_free(flat.buf);
	}
}

static void event_listener(
	esp_event_base_t event_base, int32_t event_id, void *event_data
) {
	if (event_base == WIFI_EVENT) {
		STORED_LOGF("WIFI event: %d", event_id);
	} else if (event_base == IP_EVENT) {
		STORED_LOGF("IP event: %d", event_id);
	} else {
		STORED_LOGF("Unknown event base %p, id: %d", event_base, event_id);
	}

	if (event_base == WIFI_EVENT) {
		switch (event_id) {
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
					bool is_expected_disconnect =
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
						STORED_LOGF("returned ENC_SYM(symbol_wrong_password) "
									"to the blocked thread");
						is_waiting = false;
						lbm_unblock_ctx_unboxed(
							waiting_cid, ENC_SYM(symbol_wrong_password)
						);
					} else if (!is_expected_disconnect) {
						STORED_LOGF("returned ENC_SYM_NIL to the blocked thread"
						);
						is_waiting = false;
						lbm_unblock_ctx_unboxed(waiting_cid, ENC_SYM_NIL);
					}
				}
				break;
			}
		}
	} else if (event_base == IP_EVENT) {
		switch (event_id) {
			case IP_EVENT_STA_GOT_IP: {
				if (is_waiting && waiting_op == WAITING_OP_CHANGE_NETWORK) {
					STORED_LOGF("returned ENC_SYM_TRUE to the blocked thread");
					is_waiting = false;
					lbm_unblock_ctx_unboxed(waiting_cid, ENC_SYM_TRUE);
				}
				break;
			}
		}
	}
}

static void socket_op_return_unboxed(lbm_cid return_cid, lbm_value value) {
	STORED_LOGF("returning value to cid: %d", return_cid);

	if (!lbm_unblock_ctx_unboxed(return_cid, value)) {
		STORED_LOGF("lbm_unblock_ctx_unboxed failed");
	} else {
		STORED_LOGF("socket_op_return succeeded");
	}

	socket_is_waiting = false;
	free_socket_task();
}
static void socket_op_return_flat(lbm_cid return_cid, lbm_flat_value_t *value) {
	STORED_LOGF("returning flat value to cid: %d", return_cid);

	if (!lbm_unblock_ctx(return_cid, value)) {
		STORED_LOGF("lbm_unblock_ctx failed, value: %p", value);
	} else {
		STORED_LOGF("socket_op_return_flat succeeded");
	}

	socket_is_waiting = false;
	free_socket_task();
}

// clang-format doesn't seem to understand _Generic expressions...
#define socket_op_return(value)                                                                                                                    \
	{                                                                                                                                              \
		_Generic((value), lbm_value: socket_op_return_unboxed, unsigned int: socket_op_return_unboxed, lbm_flat_value_t *: socket_op_return_flat)( \
			return_cid, value                                                                                                                      \
		);                                                                                                                                         \
		return;                                                                                                                                    \
	}

static void socket_op_recv(
	lbm_cid return_cid, int sock, size_t max_len, float timeout_secs,
	bool as_str
) {
	// Damn this code sure ain't pretty...

	uint8_t buffer[max_len + 1];

	int start = xTaskGetTickCount();
	do {
		ssize_t len = recv(sock, buffer, max_len, MSG_DONTWAIT);

		if (len < 0) {
			STORED_LOGF("errno: %d", errno);

			switch (errno) {
				case EWOULDBLOCK: {
					// TODO: Is this a good sleep interval?
					vTaskDelay(10 / portTICK_PERIOD_MS);
					continue;
				}
				case ECONNRESET:
				case ECONNABORTED:
				case ENOTCONN: {
					socket_op_return(ENC_SYM(symbol_disconnected));
				}
				default: {
					// an error has occurred
					STORED_LOGF("recv in socket_task failed, errno: %d", errno);
					socket_op_return(ENC_SYM_NIL);
				}
			}
		} else {
			STORED_LOGF("received %d bytes in socket_task", len);

			if (len == 0) {
				socket_op_return(ENC_SYM(symbol_disconnected));
			} else {
				size_t result_size = len;
				if (as_str) {
					result_size++;
					buffer[len] = '\0';
				}

				STORED_LOGF(
					"packing flat value for array size: %u", result_size
				);
				lbm_flat_value_t value;
				if (!f_pack_array(&value, buffer, result_size)) {
					STORED_LOGF("lbm_start_flatten failed");
					socket_op_return(ENC_SYM_EERROR);
				}

				socket_op_return(&value);
			}
		}
	} while (UTILS_AGE_S(start) < timeout_secs);

	STORED_LOGF("timed out after %d seconds", (double)UTILS_AGE_S(start));

	socket_op_return(ENC_SYM(symbol_no_data));
}

static void socket_task(void *arg) {
	(void)arg;

	while (true) {
		vTaskDelay(10 / portTICK_PERIOD_MS);

		if (!socket_is_waiting) {
			continue;
		}

		switch (socket_op) {
			case SOCKET_OP_RECV: {
				socket_op_recv(
					socket_waiting_cid, socket_param_sock, socket_param_max_len,
					socket_param_timeout_secs, socket_param_as_str
				);
			}
		}
	}
}

static bool scan_results_ready = false;
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
	if (!check_mode()) {
		return ENC_SYM_EERROR;
	}

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

	if (!scan_results_ready) {
		// See documentation for config parameters:
		// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#scan-configuration
		wifi_scan_config_t config = {
			.bssid       = NULL,
			.ssid        = NULL,
			.channel     = channel,
			.scan_type   = WIFI_SCAN_TYPE_PASSIVE,
			.show_hidden = show_hidden,
			.scan_time =
				{.active = {scan_time, scan_time}, .passive = scan_time},
		};

		esp_err_t result = esp_wifi_scan_start(&config, true);
		switch (result) {
			case ESP_OK: {
				break;
			}
			case ESP_ERR_WIFI_NOT_STARTED: {
				// Should not be possible.
				return ENC_SYM_FATAL_ERROR;
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
	}

	uint16_t len;
	{
		esp_err_t result = esp_wifi_scan_get_ap_num(&len);
		switch (result) {
			case ESP_OK: {
				break;
			}
			case ESP_ERR_INVALID_ARG: {
				STORED_LOGF("esp_wifi_scan_get_ap_num returned "
							"ESP_ERR_INVALID_ARG! :O");
				esp_wifi_clear_ap_list();
				return ENC_SYM_EERROR;
			}
			case ESP_ERR_WIFI_NOT_INIT:
			case ESP_ERR_WIFI_NOT_STARTED:
			default: {
				STORED_LOGF(
					"esp_wifi_scan_get_ap_num failed, result: %d", result
				);
				esp_wifi_clear_ap_list();
				return ENC_SYM_EERROR;
			}
		}
	}

	bool allocate_success = true;
	lbm_value ssid_list   = lbm_allocate_empty_list_grid(len, 3);
	allocate_success      = ssid_list != ENC_SYM_MERROR;

	lbm_value ssid_buffers[len];
	if (allocate_success) {
		for (uint16_t i = 0; i < len; i++) {
			if (!lbm_heap_allocate_array(&ssid_buffers[i], SSID_SIZE)) {
				allocate_success = false;
				break;
			}
		}
	}

	if (!allocate_success) {
		if (scan_results_ready == false) {
			// We can only retry once.
			scan_results_ready = true;
		} else {
			// Cleanup since we failed twice.
			scan_results_ready = false;
			esp_wifi_clear_ap_list();
		}

		return ENC_SYM_MERROR;
	}

	for (uint16_t i = 0; i < len; i++) {
		lbm_value ssid_str = ssid_buffers[i];
		uint8_t *data      = lbm_heap_array_get_data(ssid_str);
		if (data == NULL) {
			STORED_LOGF(
				"pre check invalid ssid_buffers[%u] data pointer "
				"%p, "
				"lbm_is_array_r(ssid_str): %u, "
				"lbm_is_array_rw(ssid_str): %u, "
				"type_of: 0x%x",
				i, data, lbm_is_array_r(ssid_str), lbm_is_array_rw(ssid_str),
				lbm_type_of(ssid_str)
			);
			return false;
		}
	}

	{
		wifi_ap_record_t records[len];
		uint16_t written_len = len;
		esp_err_t result = esp_wifi_scan_get_ap_records(&written_len, records);
		switch (result) {
			case ESP_OK: {
				break;
			}
			case ESP_ERR_NO_MEM: {
				lbm_set_error_reason(error_esp_no_memory);
				// TODO: Is this necessary?
				esp_wifi_clear_ap_list();
				return ENC_SYM_FATAL_ERROR;
			}
			default: {
				// TODO: Is this necessary?
				esp_wifi_clear_ap_list();
				return ENC_SYM_EERROR;
			}
		}

		load_lbm_list_with_ap_records(
			written_len, ssid_list, ssid_buffers, records
		);
	}

	return ssid_list;
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
	if (!check_mode()) {
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
	if (!check_mode()) {
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
	if (!check_mode()) {
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
	if (!check_mode()) {
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
	if (!check_mode() || !lock_socket_task()) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 2)) {
		free_socket_task();
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_array_r(args[0]) || !lbm_is_number(args[1])) {
		free_socket_task();
		return ENC_SYM_TERROR;
	}

	const char *host = lbm_dec_str(args[0]);
	if (!host) {
		// should be impossible
		free_socket_task();
		return ENC_SYM_FATAL_ERROR;
	}

	const uint16_t port = lbm_dec_as_u32(args[1]);

	ip_addr_t ip_addr;
	{
		err_t result = netconn_gethostbyname(host, &ip_addr);
		if (result != ERR_OK) {
			STORED_LOGF("netconn_gethostbyname failed, result: %d", result);
			free_socket_task();
			return ENC_SYM(symbol_unknown_host);
		}
	}

	struct sockaddr_in addr = create_sockaddr_in(ip_addr, port);

	int sock = comm_wifi_open_socket();
	if (sock < 0) {
		STORED_LOGF("comm_wifi_open_socket failed, result: %d", sock);
		free_socket_task();
		return ENC_SYM_NIL;
	}

	{
		int result = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
		if (result != 0) {
			STORED_LOGF("connect failed, result: %d, errno: %d", result, errno);

			shutdown(sock, 0);
			close(sock);
			free_socket_task();
			return ENC_SYM_NIL;
		}
	}

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

	free_socket_task();
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
	if (!check_mode() || !lock_socket_task()) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 1)) {
		free_socket_task();
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0])) {
		free_socket_task();
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	if (!comm_wifi_close_socket(sock)) {
		free_socket_task();
		return ENC_SYM_NIL;
	}

	free_socket_task();
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
	if (!check_mode() || !lock_socket_task()) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 1)) {
		free_socket_task();
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0])) {
		free_socket_task();
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	if (!comm_wifi_socket_is_valid(sock)) {
		STORED_LOGF("socket %d did not exist in registry", sock);
		free_socket_task();
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
		STORED_LOGF("recv for getting status failed, errno: %d", errno);
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
				free_socket_task();
				return ENC_SYM_NIL;
			}
		}
	}

	free_socket_task();
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
	if (!check_mode() || !lock_socket_task()) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn(argn, 2)) {
		free_socket_task();
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_array_r(args[1])) {
		free_socket_task();
		return ENC_SYM_TERROR;
	}

	int sock = lbm_dec_as_i32(args[0]);

	const lbm_array_header_t *array = lbm_dec_array_header(args[1]);
	size_t size                     = (size_t)array->size;
	const char *data                = (char *)array->data;
	if (!array || !array->data) {
		// Should be impossible.
		free_socket_task();
		return ENC_SYM_FATAL_ERROR;
	}

	ssize_t len = send(sock, data, size, 0);
	if (len == -1) {
		STORED_LOGF("send failed, errno: %d", errno);
		free_socket_task();
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

	STORED_LOGF("sent %d bytes", len);

	free_socket_task();
	return ENC_SYM_TRUE;
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
	if (!check_mode() || !lock_socket_task()) {
		return ENC_SYM_EERROR;
	}

	if (!lbm_check_argn_range(argn, 2, 4)) {
		free_socket_task();
		return ENC_SYM_EERROR;
	}

	if (!lbm_is_number(args[0]) || !lbm_is_number(args[1])) {
		free_socket_task();
		return ENC_SYM_TERROR;
	}

	int sock       = lbm_dec_as_i32(args[0]);
	size_t max_len = lbm_dec_as_u32(args[1]);

	bool should_wait   = true;
	float timeout_secs = 1.0; // assign to get compiler to shut up
	if (argn >= 3) {
		if (!lbm_is_number(args[2]) && !lbm_is_symbol_nil(args[2])) {
			free_socket_task();
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
			free_socket_task();
			return ENC_SYM_TERROR;
		}

		as_str = lbm_dec_bool(args[3]);
	}

	if (should_wait) {
		lbm_block_ctx_from_extension();

		socket_waiting_cid        = lbm_get_current_cid();
		socket_op                 = SOCKET_OP_RECV;
		socket_param_max_len      = max_len;
		socket_param_timeout_secs = timeout_secs;
		socket_param_sock         = sock;
		socket_param_as_str       = as_str;
		socket_is_waiting         = true;
		STORED_LOGF("socket_waiting_cid: %d", socket_waiting_cid);

		return ENC_SYM_NIL;
	} else {
		lbm_value result;
		size_t size = max_len;
		if (as_str) {
			size++;
		}
		if (!lbm_create_array(&result, size)) {
			free_socket_task();
			return ENC_SYM_MERROR;
		}

		char *buffer = lbm_dec_array_data(result);
		if (!buffer) {
			// impossible
			free_socket_task();
			return ENC_SYM_FATAL_ERROR;
		}

		ssize_t len = recv(sock, buffer, max_len, MSG_DONTWAIT);
		if (len == -1) {
			STORED_LOGF("recv failed, errno: %d", errno);
			free_socket_task();
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

		STORED_LOGF("got data of len %d", len);

		if (len == 0) {
			// Receiving 0 bytes seems to happen right before
			// getting a ENOTCONN error, which means that the remote
			// has closed the connection. (You might also get 0
			// bytes if the local code has called shutdown somewhere
			// I think, but since these lbm APIs don't do that, we
			// can ignore that case.)
			free_socket_task();
			return ENC_SYM(symbol_disconnected);
		} else {
			size_t size = len;
			if (as_str) {
				size++;
				buffer[len] = '\0';
			}
			lbm_array_shrink(result, size);
		}

		free_socket_task();
		return result;
	}
}

void lispif_load_wifi_extensions(void) {
	socket_mutex = xSemaphoreCreateBinary();
	if (socket_mutex == NULL) {
		// oh nose...
	}

	xSemaphoreGive(socket_mutex);
	xTaskCreatePinnedToCore(
		socket_task, "lbm_sockets", 2048, NULL, 3, NULL, tskNO_AFFINITY
	);
	comm_wifi_set_event_listener(event_listener);

	register_symbols();

	lbm_add_extension("wifi-scan-networks", ext_wifi_scan_networks);
	lbm_add_extension("wifi-connect", ext_wifi_connect);
	lbm_add_extension("wifi-disconnect", ext_wifi_disconnect);
	lbm_add_extension("wifi-status", ext_wifi_status);
	lbm_add_extension("wifi-auto-reconnect", ext_wifi_auto_reconnect);
	lbm_add_extension("tcp-connect", ext_tcp_connect);
	lbm_add_extension("tcp-close", ext_tcp_close);
	lbm_add_extension("tcp-status", ext_tcp_status);
	lbm_add_extension("tcp-send", ext_tcp_send);
	lbm_add_extension("tcp-recv", ext_tcp_recv);
}