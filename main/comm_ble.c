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

#include "comm_ble.h"
#include <stdlib.h>
#include <string.h>
#include "packet.h"
#include "commands.h"
#include "conf_general.h"
#include "main.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs_mbuf.h"
#include "ble_hs_priv.h"
#include "ble/custom_ble.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/util/util.h"
#include "store/config/ble_store_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "esp_hosted.h"
#include "esp_hosted_bt_host_stack.h"
#endif
#if !CONFIG_IDF_TARGET_ESP32P4
#include "esp_bt.h"
#endif

#define GATTS_CHAR_VAL_LEN_MAX 255
#define DEFAULT_BLE_MTU        20

typedef struct {
	unsigned int len;
	unsigned int pos;
	uint8_t data[];
} ble_tx_packet_t;

static QueueHandle_t send_queue;
static struct ble_npl_callout send_callout;
static ble_tx_packet_t *send_current;
static SemaphoreHandle_t send_mutex;
static bool is_connected;
static bool notify_enabled;
static uint16_t ble_current_mtu = DEFAULT_BLE_MTU;
static uint16_t notify_conn_id = BLE_HS_CONN_HANDLE_NONE;
static uint16_t char2_handle;
static PACKET_STATE_t *packet_state;
static uint8_t *char1_str;
static uint8_t *char2_str;
static uint16_t char2_len = GATTS_CHAR_VAL_LEN_MAX;

static const ble_uuid128_t ble_service_uuid128 = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t char1_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t char2_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

// NimBLE's store initialization is exported without a public declaration.
void ble_store_config_init(void);

static volatile bool has_synced;
static uint8_t own_addr_type;
static ble_event_cb_t event_cb;
static TaskHandle_t host_task;

typedef struct {
	struct ble_npl_event event;
	SemaphoreHandle_t done;
	int (*callback)(void *arg);
	void *arg;
	int result;
} ble_call_t;

static int char1_write_handler(uint16_t conn_id, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	(void)conn_id;
	(void)attr_handle;
	(void)arg;
	uint16_t len;
	if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
		return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
	}
	if (ble_hs_mbuf_to_flat(ctxt->om, char1_str, GATTS_CHAR_VAL_LEN_MAX, &len) != 0) {
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}
	for (int i = 0; i < len; i++) {
		packet_process_byte(char1_str[i], packet_state);
	}
	return 0;
}

static int char2_read_handler(uint16_t conn_id, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	(void)conn_id;
	(void)attr_handle;
	(void)arg;
	return os_mbuf_append(ctxt->om, char2_str, char2_len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static void start_advertising(void) {
	if (backup.config.ble_mode == BLE_MODE_DISABLED || is_connected || custom_ble_started()) {
		return;
	}
	uint8_t adv_data[21] = { 2, BLE_HS_ADV_TYPE_FLAGS, BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP, 17,
		BLE_HS_ADV_TYPE_COMP_UUIDS128 };
	memcpy(adv_data + 5, ble_service_uuid128.value, 16);
	uint8_t scan_rsp_data[31];
	size_t len = strnlen((char *)backup.config.ble_name, sizeof(backup.config.ble_name));
	scan_rsp_data[0] = len + 1;
	scan_rsp_data[1] = BLE_HS_ADV_TYPE_COMP_NAME;
	memcpy(scan_rsp_data + 2, (const char *)backup.config.ble_name, len);
	struct ble_gap_adv_params ble_adv_params = {
		.conn_mode = BLE_GAP_CONN_MODE_UND,
		.disc_mode = BLE_GAP_DISC_MODE_GEN,
		.itvl_min = 0x20,
		.itvl_max = 0x40,
	};
	int res = comm_ble_host_advertise(&ble_adv_params, adv_data, sizeof(adv_data), scan_rsp_data, len + 2);
	if (res != 0) {
		commands_printf("BLE advertising failed: %d", res);
	}
}

static void send_queued_packets(struct ble_npl_event *event) {
	(void)event;
	if (!is_connected || !notify_enabled || char2_handle == 0) {
		free(send_current);
		send_current = NULL;
		ble_tx_packet_t *packet;
		while (xQueueReceive(send_queue, &packet, 0) == pdTRUE) {
			free(packet);
		}
		return;
	}
	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		struct ble_gap_conn_desc desc;
		if (ble_gap_conn_find(notify_conn_id, &desc) != 0 || !desc.sec_state.encrypted || !desc.sec_state.authenticated) {
			ble_npl_callout_reset(&send_callout, 1);
			return;
		}
	}
	if (!send_current && xQueueReceive(send_queue, &send_current, 0) != pdTRUE) {
		return;
	}
	uint16_t mtu = ble_att_mtu(notify_conn_id);
	ble_current_mtu = mtu > 3 ? mtu - 3 : DEFAULT_BLE_MTU;
	if (ble_current_mtu > GATTS_CHAR_VAL_LEN_MAX) {
		ble_current_mtu = GATTS_CHAR_VAL_LEN_MAX;
	}
	unsigned int remaining = send_current->len - send_current->pos;
	uint16_t length = remaining > ble_current_mtu ? ble_current_mtu : remaining;
	uint8_t *data = send_current->data + send_current->pos;
	struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
	if (om && ble_gatts_notify_custom(notify_conn_id, char2_handle, om) == 0) {
		memcpy(char2_str, data, length);
		char2_len = length;
		send_current->pos += length;
		if (send_current->pos == send_current->len) {
			free(send_current);
			send_current = NULL;
		}
	}
	// Yield to the host between fragments. Retry when transport buffers are full.
	ble_npl_callout_reset(&send_callout, 1);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
	(void)arg;
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status != 0) {
				start_advertising();
				break;
			}
			if (is_connected) {
				ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
				return 0;
			}
			notify_conn_id = event->connect.conn_handle;
			ble_current_mtu = DEFAULT_BLE_MTU;
			is_connected = true;
			LED_BLUE_ON();
			if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
				comm_ble_host_secure(notify_conn_id);
			}
			break;
		case BLE_GAP_EVENT_DISCONNECT:
			if (event->disconnect.conn.conn_handle != notify_conn_id) {
				return 0;
			}
			is_connected = false;
			notify_enabled = false;
			notify_conn_id = BLE_HS_CONN_HANDLE_NONE;
			ble_current_mtu = DEFAULT_BLE_MTU;
			if (packet_state) {
				packet_reset(packet_state);
				send_queued_packets(NULL);
			}
			LED_BLUE_OFF();
			start_advertising();
			break;
		case BLE_GAP_EVENT_MTU:
			if (event->mtu.channel_id == BLE_L2CAP_CID_ATT) {
				uint16_t mtu = event->mtu.value > 3 ? event->mtu.value - 3 : DEFAULT_BLE_MTU;
				ble_current_mtu = mtu > GATTS_CHAR_VAL_LEN_MAX ? GATTS_CHAR_VAL_LEN_MAX : mtu;
			}
			break;
		case BLE_GAP_EVENT_SUBSCRIBE:
			if (event->subscribe.attr_handle == char2_handle) {
				notify_enabled = event->subscribe.cur_notify;
			}
			break;
		case BLE_GAP_EVENT_ADV_COMPLETE:
			start_advertising();
			break;
		default:
			break;
	}
	if (custom_ble_started()) {
		return custom_ble_gap_event(event, arg);
	}
	return 0;
}

uint16_t comm_ble_conn_handle(void) {
	return notify_conn_id;
}

bool comm_ble_service_uuid_reserved(const ble_uuid_t *uuid) {
	return ble_uuid_cmp(uuid, &ble_service_uuid128.u) == 0;
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_ble_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	if (!is_connected || !notify_enabled) {
		return;
	}
	ble_tx_packet_t *packet = malloc(sizeof(*packet) + len);
	if (!packet) {
		return;
	}
	packet->len = len;
	packet->pos = 0;
	memcpy(packet->data, buffer, len);
	if (xQueueSend(send_queue, &packet, 0) != pdTRUE) {
		free(packet);
		return;
	}
	ble_npl_callout_reset(&send_callout, 1);
}

static struct ble_gatt_chr_def ble_chars[] = {
	{
		.uuid = &char1_uuid.u,
		.access_cb = char1_write_handler,
		.flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
	},
	{
		.uuid = &char2_uuid.u,
		.access_cb = char2_read_handler,
		.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.val_handle = &char2_handle,
	},
	{ 0 },
};

static const struct ble_gatt_svc_def ble_services[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &ble_service_uuid128.u,
		.characteristics = ble_chars,
	},
	{ 0 },
};

static void free_packet_resources(void) {
	free(packet_state);
	packet_state = NULL;
	free(char1_str);
	char1_str = NULL;
	char2_str = NULL;
	if (send_queue) {
		vQueueDelete(send_queue);
		send_queue = NULL;
	}
	if (send_mutex) {
		vSemaphoreDelete(send_mutex);
		send_mutex = NULL;
	}
}

void comm_ble_init(void) {
	bool vesc_enabled = backup.config.ble_mode != BLE_MODE_DISABLED;
	if (vesc_enabled) {
		send_mutex = xSemaphoreCreateMutex();
		// A Lisp error report can enqueue several packets before the host drains it.
		send_queue = xQueueCreate(32, sizeof(ble_tx_packet_t *));

		packet_state = calloc(1, sizeof(PACKET_STATE_t));
		char1_str = calloc(2, GATTS_CHAR_VAL_LEN_MAX);
		if (!packet_state || !char1_str || !send_mutex || !send_queue) {
			free_packet_resources();
			return;
		}
		char2_str = char1_str + GATTS_CHAR_VAL_LEN_MAX;
		packet_init(send_packet_raw, process_packet, packet_state);
	}
	bool encrypted = backup.config.ble_mode == BLE_MODE_ENCRYPTED;
	if (encrypted) {
		ble_chars[0].flags |= BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN;
		ble_chars[1].flags |= BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN |
			BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC;
	}
	bool server_enabled = vesc_enabled || backup.config.ble_mode_lbm == BLE_MODE_LBM_SERVER ||
		backup.config.ble_mode_lbm == BLE_MODE_LBM_CLIENT_SERVER;
	int res = comm_ble_host_init((char *)backup.config.ble_name, encrypted, server_enabled ? gap_event_handler : NULL);
	if (res == 0 && vesc_enabled) {
		ble_npl_callout_init(&send_callout, nimble_port_get_dflt_eventq(), send_queued_packets, NULL);
		res = ble_gatts_count_cfg(ble_services);
	}
	if (res == 0 && vesc_enabled) {
		res = ble_gatts_add_svcs(ble_services);
	}
	if (res == 0) {
		res = custom_ble_reserve_resources();
	}
	if (res != 0) {
		commands_printf("BLE initialization failed: %d", res);
		free_packet_resources();
		return;
	}
	comm_ble_host_start();
}

bool comm_ble_is_connected(void) {
	return is_connected;
}

int comm_ble_mtu_now(void) {
	return ble_current_mtu;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	if (packet_state) {
		xSemaphoreTake(send_mutex, portMAX_DELAY);
		packet_send_packet(data, len, packet_state);
		xSemaphoreGive(send_mutex);
	}
}

static void ble_call_handler(struct ble_npl_event *event) {
	ble_call_t *call = ble_npl_event_get_arg(event);
	call->result = call->callback(call->arg);
	xSemaphoreGive(call->done);
}

int comm_ble_host_call(int (*callback)(void *arg), void *arg) {
	if (xTaskGetCurrentTaskHandle() == host_task) {
		return callback(arg);
	}
	ble_call_t call = { .callback = callback, .arg = arg };
	call.done = xSemaphoreCreateBinary();
	if (!call.done) {
		return BLE_HS_ENOMEM;
	}
	ble_npl_event_init(&call.event, ble_call_handler, &call);
	ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &call.event);
	// The event owns the stack arguments until it completes.
	xSemaphoreTake(call.done, portMAX_DELAY);
	ble_npl_event_deinit(&call.event);
	vSemaphoreDelete(call.done);
	return call.result;
}

static void ble_on_sync(void) {
	if (ble_hs_util_ensure_addr(0) == 0 && ble_hs_id_infer_auto(0, &own_addr_type) == 0) {
		has_synced = true;
		if (event_cb) {
			struct ble_gap_event event = { .type = BLE_GAP_EVENT_ADV_COMPLETE };
			event_cb(&event, NULL);
		}
	}
}

static void ble_on_reset(int reason) {
	(void)reason;
	has_synced = false;
}

int comm_ble_host_secure(uint16_t connection) {
	ble_hs_lock();
	bool pending = ble_sm_proc_find(connection, BLE_SM_PROC_STATE_NONE, -1, NULL) != NULL;
	ble_hs_unlock();
	return pending ? 0 : ble_gap_security_initiate(connection);
}

int comm_ble_host_init(const char *name, bool encrypted, ble_event_cb_t callback) {
#if CONFIG_IDF_TARGET_ESP32P4
	int res = esp_hosted_connect_to_slave();
	if (res != 0) {
		return res;
	}
	esp_hosted_bt_host_stack_cfg_t bt_cfg = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
	res = esp_hosted_bt_host_stack_setup(&bt_cfg);
	if (res != 0) {
		return res;
	}
	res = nimble_port_init();
#else
	int res = nimble_port_init();
#endif
	if (res != 0) {
		return res;
	}
	event_cb = callback;
	ble_hs_cfg.sync_cb = ble_on_sync;
	ble_hs_cfg.reset_cb = ble_on_reset;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	ble_hs_cfg.sm_io_cap = encrypted ? BLE_HS_IO_DISPLAY_ONLY : BLE_HS_IO_NO_INPUT_OUTPUT;
	ble_hs_cfg.sm_bonding = encrypted;
	ble_hs_cfg.sm_mitm = encrypted;
	ble_hs_cfg.sm_sc = 1;
	ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
	ble_store_config_init();
	if (callback) {
		ble_svc_gap_init();
		ble_svc_gatt_init();
		res = ble_svc_gap_device_name_set(name);
	}
	if (res == 0) {
		res = ble_att_set_preferred_mtu(256);
	}
#if !CONFIG_IDF_TARGET_ESP32P4
#if CONFIG_IDF_TARGET_ESP32
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
#else
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P18);
#endif
#endif
	if (res != 0) {
		nimble_port_deinit();
	}
	return res;
}

static void ble_host_task(void *arg) {
	(void)arg;
	host_task = xTaskGetCurrentTaskHandle();
	nimble_port_run();
	nimble_port_freertos_deinit();
}

void comm_ble_host_start(void) {
	nimble_port_freertos_init(ble_host_task);
}

bool comm_ble_host_ready(void) {
	return has_synced;
}

static int ble_host_gap_event_handler(struct ble_gap_event *event, void *arg) {
	if (event->type == BLE_GAP_EVENT_REPEAT_PAIRING) {
		struct ble_gap_conn_desc desc;
		if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) != 0 ||
				ble_store_util_delete_peer(&desc.peer_id_addr) != 0) {
			return BLE_GAP_REPEAT_PAIRING_IGNORE;
		}
		return BLE_GAP_REPEAT_PAIRING_RETRY;
	}
	if (event->type == BLE_GAP_EVENT_PASSKEY_ACTION && event->passkey.params.action == BLE_SM_IOACT_DISP) {
		struct ble_sm_io io = {
			.action = BLE_SM_IOACT_DISP,
			.passkey = backup.config.ble_pin,
		};
		return ble_sm_inject_io(event->passkey.conn_handle, &io);
	}
	return event_cb ? event_cb(event, arg) : 0;
}

int comm_ble_host_advertise(struct ble_gap_adv_params *params, const uint8_t *adv_data, size_t adv_len,
	const uint8_t *scan_rsp_data, size_t scan_rsp_len) {
	if (!has_synced) {
		return BLE_HS_ENOTSYNCED;
	}
	int res = ble_gap_adv_set_data(adv_data, adv_len);
	if (res == 0) {
		res = ble_gap_adv_rsp_set_data(scan_rsp_data, scan_rsp_len);
	}
	if (res == 0) {
		res = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, params, ble_host_gap_event_handler, NULL);
	}
	return res;
}
#else
void comm_ble_init(void) {
}
bool comm_ble_is_connected(void) {
	return false;
}
int comm_ble_mtu_now(void) {
	return 0;
}
void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}
#endif
