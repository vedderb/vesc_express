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
#include "lispif.h"
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
// Upper bound for backup.config.ble_max_conn. Each slot needs its own reply function.
#define BLE_CONN_SLOTS         4

typedef struct {
	unsigned int len;
	unsigned int pos;
	uint8_t data[];
} ble_tx_packet_t;

// One incoming VESC connection. Slots below conn_slots are allocated at init.
typedef struct {
	uint16_t conn;
	bool notify_enabled;
	uint16_t mtu;
	PACKET_STATE_t *packet_state;
	QueueHandle_t send_queue;
	ble_tx_packet_t *send_current;
	SemaphoreHandle_t send_mutex;
} ble_conn_t;

static ble_conn_t conns[BLE_CONN_SLOTS];
static int conn_slots;
static struct ble_npl_callout send_callout;
static uint16_t char2_handle;
static uint8_t *char1_str;
static uint8_t *char2_str;
static uint16_t char2_len = GATTS_CHAR_VAL_LEN_MAX;

static void slot_send_raw(ble_conn_t *c, unsigned char *buffer, unsigned int len);
static void slot_send_packet(ble_conn_t *c, unsigned char *data, unsigned int len);

// send_func_t has no context argument, so every slot gets its own functions. commands.c
// keys reply routing on the function pointer, so each connection must have a distinct one.
#define SLOT_FUNCS(i) \
	static void slot_reply_##i(unsigned char *d, unsigned int l) { slot_send_packet(&conns[i], d, l); } \
	static void slot_raw_##i(unsigned char *d, unsigned int l) { slot_send_raw(&conns[i], d, l); } \
	static void slot_process_##i(unsigned char *d, unsigned int l) { commands_process_packet(d, l, slot_reply_##i); }
SLOT_FUNCS(0)
SLOT_FUNCS(1)
SLOT_FUNCS(2)
SLOT_FUNCS(3)

static void (*const slot_reply[BLE_CONN_SLOTS])(unsigned char *, unsigned int) =
	{ slot_reply_0, slot_reply_1, slot_reply_2, slot_reply_3 };
static void (*const slot_raw[BLE_CONN_SLOTS])(unsigned char *, unsigned int) =
	{ slot_raw_0, slot_raw_1, slot_raw_2, slot_raw_3 };
static void (*const slot_process[BLE_CONN_SLOTS])(unsigned char *, unsigned int) =
	{ slot_process_0, slot_process_1, slot_process_2, slot_process_3 };

static ble_conn_t *slot_find(uint16_t conn) {
	for (int i = 0; i < conn_slots; i++) {
		if (conns[i].conn == conn && conn != BLE_HS_CONN_HANDLE_NONE) {
			return &conns[i];
		}
	}
	return NULL;
}

static int active_count(void) {
	int n = 0;
	for (int i = 0; i < conn_slots; i++) {
		if (conns[i].conn != BLE_HS_CONN_HANDLE_NONE) {
			n++;
		}
	}
	return n;
}

// The LBM BLE server (custom_ble.c) tracks a single connection.
static int active_limit(void) {
	return custom_ble_started() ? 1 : conn_slots;
}

static void slot_flush(ble_conn_t *c) {
	free(c->send_current);
	c->send_current = NULL;
	ble_tx_packet_t *packet;
	while (c->send_queue && xQueueReceive(c->send_queue, &packet, 0) == pdTRUE) {
		free(packet);
	}
}

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
	(void)attr_handle;
	(void)arg;
	uint16_t len;
	if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
		return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
	}
	ble_conn_t *c = slot_find(conn_id);
	if (!c) {
		return BLE_ATT_ERR_UNLIKELY;
	}
	if (ble_hs_mbuf_to_flat(ctxt->om, char1_str, GATTS_CHAR_VAL_LEN_MAX, &len) != 0) {
		return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
	}
	for (int i = 0; i < len; i++) {
		packet_process_byte(char1_str[i], c->packet_state);
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
	if (backup.config.ble_mode == BLE_MODE_DISABLED || conn_slots == 0 || active_count() >= active_limit() ||
			custom_ble_started() || ble_gap_adv_active()) {
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

// Sends one fragment for a slot. Returns true if the slot needs another pass.
static bool slot_service(ble_conn_t *c) {
	if (c->conn == BLE_HS_CONN_HANDLE_NONE || !c->notify_enabled || char2_handle == 0) {
		slot_flush(c);
		return false;
	}
	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		struct ble_gap_conn_desc desc;
		if (ble_gap_conn_find(c->conn, &desc) != 0 || !desc.sec_state.encrypted || !desc.sec_state.authenticated) {
			return uxQueueMessagesWaiting(c->send_queue) > 0 || c->send_current;
		}
	}
	if (!c->send_current && xQueueReceive(c->send_queue, &c->send_current, 0) != pdTRUE) {
		return false;
	}
	uint16_t mtu = ble_att_mtu(c->conn);
	c->mtu = mtu > 3 ? mtu - 3 : DEFAULT_BLE_MTU;
	if (c->mtu > GATTS_CHAR_VAL_LEN_MAX) {
		c->mtu = GATTS_CHAR_VAL_LEN_MAX;
	}
	unsigned int remaining = c->send_current->len - c->send_current->pos;
	uint16_t length = remaining > c->mtu ? c->mtu : remaining;
	uint8_t *data = c->send_current->data + c->send_current->pos;
	struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
	if (om && ble_gatts_notify_custom(c->conn, char2_handle, om) == 0) {
		memcpy(char2_str, data, length);
		char2_len = length;
		c->send_current->pos += length;
		if (c->send_current->pos == c->send_current->len) {
			free(c->send_current);
			c->send_current = NULL;
		}
	}
	return true;
}

static void send_queued_packets(struct ble_npl_event *event) {
	(void)event;
	bool more = false;
	for (int i = 0; i < conn_slots; i++) {
		more = slot_service(&conns[i]) || more;
	}
	// Yield to the host between fragments. Retry when transport buffers are full.
	if (more) {
		ble_npl_callout_reset(&send_callout, 1);
	}
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
	(void)arg;
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT: {
			if (event->connect.status != 0) {
				start_advertising();
				break;
			}
			if (conn_slots == 0) {
				break; // VESC service disabled: the link belongs to the LBM server
			}
			ble_conn_t *c = NULL;
			if (active_count() < active_limit()) {
				for (int i = 0; i < conn_slots && !c; i++) {
					if (conns[i].conn == BLE_HS_CONN_HANDLE_NONE) {
						c = &conns[i];
					}
				}
			}
			if (!c) {
				ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
				return 0;
			}
			slot_flush(c);
			packet_reset(c->packet_state);
			c->conn = event->connect.conn_handle;
			c->notify_enabled = false;
			c->mtu = DEFAULT_BLE_MTU;
			LED_BLUE_ON();
			if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
				comm_ble_host_secure(c->conn);
			}
			// Advertising stops on connect; keep accepting until the limit is reached.
			start_advertising();
		} break;
		case BLE_GAP_EVENT_DISCONNECT: {
			if (conn_slots == 0) {
				break;
			}
			ble_conn_t *c = slot_find(event->disconnect.conn.conn_handle);
			if (!c) {
				return 0; // A link refused above
			}
			lispif_invalidate_app_data_source(slot_reply[c - conns]);
			c->conn = BLE_HS_CONN_HANDLE_NONE;
			c->notify_enabled = false;
			c->mtu = DEFAULT_BLE_MTU;
			packet_reset(c->packet_state);
			slot_flush(c);
			if (active_count() == 0) {
				LED_BLUE_OFF();
			}
			start_advertising();
		} break;
		case BLE_GAP_EVENT_MTU:
			if (event->mtu.channel_id == BLE_L2CAP_CID_ATT) {
				ble_conn_t *c = slot_find(event->mtu.conn_handle);
				if (c) {
					uint16_t mtu = event->mtu.value > 3 ? event->mtu.value - 3 : DEFAULT_BLE_MTU;
					c->mtu = mtu > GATTS_CHAR_VAL_LEN_MAX ? GATTS_CHAR_VAL_LEN_MAX : mtu;
				}
			}
			break;
		case BLE_GAP_EVENT_SUBSCRIBE:
			if (event->subscribe.attr_handle == char2_handle) {
				ble_conn_t *c = slot_find(event->subscribe.conn_handle);
				if (c) {
					c->notify_enabled = event->subscribe.cur_notify;
				}
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

// First active connection. Used by the LBM BLE server, which tracks a single connection.
uint16_t comm_ble_conn_handle(void) {
	for (int i = 0; i < conn_slots; i++) {
		if (conns[i].conn != BLE_HS_CONN_HANDLE_NONE) {
			return conns[i].conn;
		}
	}
	return BLE_HS_CONN_HANDLE_NONE;
}

bool comm_ble_service_uuid_reserved(const ble_uuid_t *uuid) {
	return ble_uuid_cmp(uuid, &ble_service_uuid128.u) == 0;
}

static void slot_send_raw(ble_conn_t *c, unsigned char *buffer, unsigned int len) {
	if (c->conn == BLE_HS_CONN_HANDLE_NONE || !c->notify_enabled) {
		return;
	}
	ble_tx_packet_t *packet = malloc(sizeof(*packet) + len);
	if (!packet) {
		return;
	}
	packet->len = len;
	packet->pos = 0;
	memcpy(packet->data, buffer, len);
	if (xQueueSend(c->send_queue, &packet, 0) != pdTRUE) {
		free(packet);
		return;
	}
	ble_npl_callout_reset(&send_callout, 1);
}

static void slot_send_packet(ble_conn_t *c, unsigned char *data, unsigned int len) {
	if (c->packet_state) {
		xSemaphoreTake(c->send_mutex, portMAX_DELAY);
		packet_send_packet(data, len, c->packet_state);
		xSemaphoreGive(c->send_mutex);
	}
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
	for (int i = 0; i < BLE_CONN_SLOTS; i++) {
		ble_conn_t *c = &conns[i];
		slot_flush(c);
		free(c->packet_state);
		c->packet_state = NULL;
		if (c->send_queue) {
			vQueueDelete(c->send_queue);
			c->send_queue = NULL;
		}
		if (c->send_mutex) {
			vSemaphoreDelete(c->send_mutex);
			c->send_mutex = NULL;
		}
		c->conn = BLE_HS_CONN_HANDLE_NONE;
	}
	conn_slots = 0;
	free(char1_str);
	char1_str = NULL;
	char2_str = NULL;
}

static int configured_max_conn(void) {
#if defined(HW_BLE_MAX_CONN)
	int n = HW_BLE_MAX_CONN;
#elif defined(OVR_CONF_MAIN_CONFIG)
	int n = 1; // Board-specific config without ble_max_conn
#else
	int n = backup.config.ble_max_conn;
#endif
	if (n < 1) {
		n = 1;
	}
	if (n > BLE_CONN_SLOTS) {
		n = BLE_CONN_SLOTS;
	}
	if (n > CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
		n = CONFIG_BT_NIMBLE_MAX_CONNECTIONS;
	}
	return n;
}

static bool alloc_packet_resources(int slots) {
	char1_str = calloc(2, GATTS_CHAR_VAL_LEN_MAX);
	if (!char1_str) {
		return false;
	}
	char2_str = char1_str + GATTS_CHAR_VAL_LEN_MAX;
	for (int i = 0; i < slots; i++) {
		ble_conn_t *c = &conns[i];
		c->conn = BLE_HS_CONN_HANDLE_NONE;
		c->mtu = DEFAULT_BLE_MTU;
		c->send_mutex = xSemaphoreCreateMutex();
		// A Lisp error report can enqueue several packets before the host drains it.
		c->send_queue = xQueueCreate(32, sizeof(ble_tx_packet_t *));
		c->packet_state = calloc(1, sizeof(PACKET_STATE_t));
		if (!c->send_mutex || !c->send_queue || !c->packet_state) {
			return false;
		}
		packet_init(slot_raw[i], slot_process[i], c->packet_state);
		conn_slots = i + 1;
	}
	return true;
}

void comm_ble_init(void) {
	for (int i = 0; i < BLE_CONN_SLOTS; i++) {
		conns[i].conn = BLE_HS_CONN_HANDLE_NONE;
	}
	bool vesc_enabled = backup.config.ble_mode != BLE_MODE_DISABLED;
	if (vesc_enabled && !alloc_packet_resources(configured_max_conn())) {
		free_packet_resources();
		return;
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
	return active_count() > 0;
}

int comm_ble_connection_count(void) {
	return active_count();
}

// Connections reserved for VESC clients. Depends only on the config, so it can be used before init.
int comm_ble_max_connections(void) {
	return backup.config.ble_mode == BLE_MODE_DISABLED ? 0 : configured_max_conn();
}

int comm_ble_mtu_now(void) {
	for (int i = 0; i < conn_slots; i++) {
		if (conns[i].conn != BLE_HS_CONN_HANDLE_NONE) {
			return conns[i].mtu;
		}
	}
	return DEFAULT_BLE_MTU;
}

// Sends to every connected client, e.g. for send-data from LispBM.
void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	for (int i = 0; i < conn_slots; i++) {
		if (conns[i].conn != BLE_HS_CONN_HANDLE_NONE) {
			slot_send_packet(&conns[i], data, len);
		}
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
int comm_ble_connection_count(void) {
	return 0;
}
int comm_ble_max_connections(void) {
	return 0;
}
int comm_ble_mtu_now(void) {
	return 0;
}
void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}
#endif
