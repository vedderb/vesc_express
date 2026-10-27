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

#include "custom_ble.h"
#include <string.h>
#include "conf_general.h"
#include "commands.h"
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "comm_ble.h"
#include "comm_wifi.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "freertos/queue.h"
#include "host/ble_hs_mbuf.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"

typedef struct {
	ble_uuid_any_t uuid;
	uint16_t attr_handle;
	uint16_t value_max_len;
	uint16_t value_len;
	uint8_t *value;
	bool notify_enabled;
	bool indicate_enabled;
	bool is_cccd;
	bool hid;
	struct ble_gatt_chr_def *chr;
} attr_instance_t;

typedef struct {
	ble_uuid_any_t uuid;
	uint16_t service_handle;
	uint16_t attr_count;
	attr_instance_t *attr;
	struct ble_gatt_svc_def *definition;
	bool hid;
} service_instance_t;

typedef struct {
	bool use_raw;
	size_t adv_len;
	const uint8_t *adv_data;
	size_t scan_rsp_len;
	const uint8_t *scan_rsp_data;
} adv_update_t;

static bool has_started;
static bool server_enabled;
static bool client_enabled;
static custom_ble_result_t init_result;
static uint16_t service_capacity;
static uint16_t chr_descr_capacity;
static uint16_t custom_service_len;
static uint16_t custom_attr_len;
static service_instance_t *custom_services;
static char device_name[CUSTOM_BLE_MAX_NAME_LEN + 1];
static attr_write_cb_t attr_write_cb;
static uint16_t conn_id = BLE_HS_CONN_HANDLE_NONE;
static SemaphoreHandle_t attr_mutex;
static bool use_custom_adv_data;
static size_t ble_adv_data_raw_len;
static uint8_t ble_adv_data_raw[31];
static size_t ble_scan_rsp_data_raw_len;
static uint8_t ble_scan_rsp_data_raw[31];

struct ble_gap_adv_params ble_adv_params = {
	.conn_mode = BLE_GAP_CONN_MODE_UND,
	.disc_mode = BLE_GAP_DISC_MODE_GEN,
	.itvl_min = 0x20,
	.itvl_max = 0x40,
	.channel_map = 7,
};

static QueueHandle_t client_events;
typedef struct {
	volatile uint16_t conn;
	bool busy;
	bool securing;
	bool require_auth;
	uint8_t passkey_action;
	uint8_t *read_value;
	uint16_t read_len;
	uint16_t read_handle;
} client_connection_t;

static client_connection_t client_connections[BLE_CLIENT_CONNECTIONS_MAX];
static unsigned int client_limit;
static bool client_connecting;
static struct ble_npl_callout client_connect_timeout;
static bool client_pair_on_connect;
static uint8_t client_pair_io;
static bool client_pair_bond;
static bool client_pair_policy_pending;
static uint8_t client_saved_io;
static bool client_saved_bond;
static bool client_saved_mitm;
static uint32_t client_dropped;

static service_instance_t *get_service(uint16_t handle) {
	for (int i = 0; i < custom_service_len; i++) {
		if (custom_services[i].service_handle == handle) {
			return &custom_services[i];
		}
	}
	return NULL;
}

static attr_instance_t *get_attr(uint16_t handle) {
	for (int i = 0; i < custom_service_len; i++) {
		service_instance_t *service = &custom_services[i];
		for (int j = 0; j < service->attr_count; j++) {
			if (service->attr[j].attr_handle == handle) {
				return &service->attr[j];
			}
		}
	}
	return NULL;
}

static void start_advertising(void) {
	if (conn_id != BLE_HS_CONN_HANDLE_NONE) {
		return;
	}
	uint8_t adv_data[31] = { 2, BLE_HS_ADV_TYPE_FLAGS, BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP, 17,
		BLE_HS_ADV_TYPE_COMP_UUIDS128, 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01,
		0x00, 0x40, 0x6E };
	size_t adv_len = backup.config.ble_mode == BLE_MODE_DISABLED ? 3 : 21;
	for (int i = 0; i < custom_service_len; i++) {
		if (custom_services[i].hid) {
			uint16_t appearance = ble_svc_gap_device_appearance();
			uint8_t hid[] = { 3, BLE_HS_ADV_TYPE_COMP_UUIDS16, 0x12, 0x18, 3, BLE_HS_ADV_TYPE_APPEARANCE, appearance, appearance >> 8 };
			memcpy(adv_data + adv_len, hid, sizeof(hid));
			adv_len += sizeof(hid);
			break;
		}
	}
	size_t len = strlen(device_name);
	// A 30-byte name fits in a scan response only when shortened to 29 bytes.
	if (len > 29) {
		len = 29;
	}
	uint8_t scan_rsp_data[31];
	scan_rsp_data[0] = len + 1;
	scan_rsp_data[1] = strlen(device_name) > len ? BLE_HS_ADV_TYPE_INCOMP_NAME : BLE_HS_ADV_TYPE_COMP_NAME;
	memcpy(scan_rsp_data + 2, device_name, len);
	int res = comm_ble_host_advertise(&ble_adv_params, use_custom_adv_data ? ble_adv_data_raw : adv_data,
		use_custom_adv_data ? ble_adv_data_raw_len : adv_len, use_custom_adv_data ? ble_scan_rsp_data_raw : scan_rsp_data,
		use_custom_adv_data ? ble_scan_rsp_data_raw_len : len + 2);
	if (res != 0) {
		commands_printf("Custom BLE advertising failed: %d", res);
	}
}

int custom_ble_gap_event(struct ble_gap_event *event, void *arg) {
	(void)arg;
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status == 0) {
				conn_id = event->connect.conn_handle;
				LED_BLUE_ON();
			} else {
				start_advertising();
			}
			break;
		case BLE_GAP_EVENT_DISCONNECT:
			conn_id = BLE_HS_CONN_HANDLE_NONE;
			xSemaphoreTake(attr_mutex, portMAX_DELAY);
			for (int i = 0; i < custom_service_len; i++) {
				for (int j = 0; j < custom_services[i].attr_count; j++) {
					attr_instance_t *attr = &custom_services[i].attr[j];
					attr->notify_enabled = false;
					attr->indicate_enabled = false;
					if (attr->is_cccd) {
						memset(attr->value, 0, 2);
					}
					if (attr->hid && ble_uuid_u16(&attr->uuid.u) == 0x2a4c) {
						attr->value[0] = 1;
					}
				}
			}
			xSemaphoreGive(attr_mutex);
			LED_BLUE_OFF();
			start_advertising();
			break;
		case BLE_GAP_EVENT_SUBSCRIBE: {
			xSemaphoreTake(attr_mutex, portMAX_DELAY);
			attr_instance_t *attr = get_attr(event->subscribe.attr_handle);
			if (attr) {
				attr->notify_enabled = event->subscribe.cur_notify;
				attr->indicate_enabled = event->subscribe.cur_indicate;
			}
			attr_instance_t *cccd = get_attr(event->subscribe.attr_handle + 1);
			uint8_t value[2] = { event->subscribe.cur_notify | (event->subscribe.cur_indicate << 1), 0 };
			uint16_t handle = 0;
			if (cccd && cccd->is_cccd) {
				memcpy(cccd->value, value, 2);
				handle = cccd->attr_handle;
			}
			xSemaphoreGive(attr_mutex);
			if (handle && attr_write_cb && event->subscribe.reason == BLE_GAP_SUBSCRIBE_REASON_WRITE) {
				attr_write_cb(handle, 2, value);
			}
			break;
		}
		case BLE_GAP_EVENT_ADV_COMPLETE:
			start_advertising();
			break;
		default:
			break;
	}
	return 0;
}

static int attr_access_handler(uint16_t connection, uint16_t handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
	(void)connection;
	attr_instance_t *attr = arg;
	bool write = ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC;
	int res = 0;
	uint16_t len = 0;
	uint8_t *value = NULL;
	xSemaphoreTake(attr_mutex, portMAX_DELAY);
	if (write) {
		len = OS_MBUF_PKTLEN(ctxt->om);
		if (len > attr->value_max_len || (attr->hid && len != attr->value_max_len)) {
			res = BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
		} else {
			value = malloc(len ? len : 1);
			if (!value) {
				res = BLE_ATT_ERR_INSUFFICIENT_RES;
			} else if (ble_hs_mbuf_to_flat(ctxt->om, value, len, &len) != 0) {
				res = BLE_ATT_ERR_UNLIKELY;
			} else {
				if (attr->hid && ble_uuid_u16(&attr->uuid.u) == 0x2a4c && value[0] > 1) {
					res = BLE_ATT_ERR_VALUE_NOT_ALLOWED;
				} else {
					memcpy(attr->value, value, len);
					attr->value_len = len;
				}
			}
		}
	} else if (os_mbuf_append(ctxt->om, attr->value, attr->value_len) != 0) {
		res = BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	xSemaphoreGive(attr_mutex);
	if (write && res == 0 && attr_write_cb) {
		attr_write_cb(handle, len, value);
	}
	free(value);
	return res;
}

static void gatts_register_handler(struct ble_gatt_register_ctxt *ctxt, void *arg) {
	(void)arg;
	// Registration runs synchronously while adding a service.
	service_instance_t *service = &custom_services[custom_service_len];
	if (ctxt->op == BLE_GATT_REGISTER_OP_SVC) {
		service->service_handle = ctxt->svc.handle;
	} else if (ctxt->op == BLE_GATT_REGISTER_OP_CHR) {
		attr_instance_t *attr = ctxt->chr.chr_def->arg;
		attr->attr_handle = ctxt->chr.val_handle;
	} else if (ctxt->op == BLE_GATT_REGISTER_OP_DSC) {
		attr_instance_t *attr = ctxt->dsc.dsc_def->arg;
		attr->attr_handle = ctxt->dsc.handle;
	}
}

static void free_service(service_instance_t *service) {
	if (service->definition) {
		struct ble_gatt_chr_def *chr = (void *)service->definition[0].characteristics;
		if (chr) {
			for (int i = 0; chr[i].uuid; i++) {
				free((void *)chr[i].descriptors);
			}
		}
		free(chr);
		free(service->definition);
	}
	if (service->attr) {
		for (int i = 0; i < service->attr_count; i++) {
			free(service->attr[i].value);
		}
	}
	free(service->attr);
	memset(service, 0, sizeof(*service));
}

static bool init_attr(
	attr_instance_t *attr, ble_uuid_any_t uuid, uint16_t max_len, uint16_t len, const uint8_t *value) {
	if (len > max_len || (len && !value)) {
		return false;
	}
	attr->uuid = uuid;
	attr->value_max_len = max_len;
	attr->value_len = len;
	attr->value = calloc(max_len ? max_len : 1, 1);
	if (!attr->value) {
		return false;
	}
	if (len) {
		memcpy(attr->value, value, len);
	}
	return true;
}

static int register_service(void *arg) {
	service_instance_t *service = arg;
	ble_hs_cfg.gatts_register_cb = gatts_register_handler;
	int res = ble_gatts_add_dynamic_svcs(service->definition);
	if (res == 0) {
		xSemaphoreTake(attr_mutex, portMAX_DELAY);
		for (int i = 0; i < service->attr_count; i++) {
			attr_instance_t *attr = &service->attr[i];
			if (attr->is_cccd) {
				attr->attr_handle = *attr->chr->val_handle + 1;
			}
		}
		custom_service_len++;
		custom_attr_len += service->attr_count;
		xSemaphoreGive(attr_mutex);
	} else if (service->service_handle && ble_gatts_delete_svc(&service->uuid.u) != 0) {
		// Keep definitions alive if the stack still references them.
		init_result = CUSTOM_BLE_INTERNAL_ERROR;
	}
	return res;
}

static int remove_service(void *arg) {
	service_instance_t *service = arg;
	bool hid = service->hid;
	int res = ble_gatts_delete_svc(&service->uuid.u);
	if (res == 0) {
		xSemaphoreTake(attr_mutex, portMAX_DELAY);
		custom_attr_len -= service->attr_count;
		custom_service_len--;
		free_service(service);
		xSemaphoreGive(attr_mutex);
		if (hid && ble_gap_adv_active()) {
			ble_gap_adv_stop();
			start_advertising();
		}
	}
	return res;
}

static int update_advertising(void *arg) {
	adv_update_t *update = arg;
	if (has_started && ble_gap_adv_active()) {
		int res = ble_gap_adv_stop();
		if (res != 0) {
			return res;
		}
	}
	use_custom_adv_data = update->use_raw;
	if (update->adv_data) {
		memcpy(ble_adv_data_raw, update->adv_data, update->adv_len);
		ble_adv_data_raw_len = update->adv_len;
	}
	if (update->scan_rsp_data) {
		memcpy(ble_scan_rsp_data_raw, update->scan_rsp_data, update->scan_rsp_len);
		ble_scan_rsp_data_raw_len = update->scan_rsp_len;
	}
	if (has_started) {
		start_advertising();
	}
	return 0;
}

static int start_script_services(void *arg) {
	(void)arg;
	int res = ble_svc_gap_device_name_set(device_name);
	if (res != 0) {
		return res;
	}
	if (ble_gap_adv_active()) {
		res = ble_gap_adv_stop();
		if (res != 0) {
			return res;
		}
	}
	conn_id = comm_ble_conn_handle();
	has_started = true;
	start_advertising();
	return 0;
}

custom_ble_result_t custom_ble_start(void) {
	if (!server_enabled) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	if (has_started) {
		return CUSTOM_BLE_ALREADY_STARTED;
	}
	if (init_result != CUSTOM_BLE_OK) {
		return CUSTOM_BLE_INIT_FAILED;
	}
	// Wait for the shared BLE host to synchronize.
	for (int i = 0; i < 100; i++) {
		if (comm_ble_host_ready()) {
			return comm_ble_host_call(start_script_services, NULL) == 0 ? CUSTOM_BLE_OK : CUSTOM_BLE_ESP_ERROR;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	return CUSTOM_BLE_TIMEOUT;
}

custom_ble_result_t custom_ble_set_name(const char *name) {
	if (has_started) {
		return CUSTOM_BLE_ALREADY_STARTED;
	}
	if (strlen(name) > CUSTOM_BLE_MAX_NAME_LEN) {
		return CUSTOM_BLE_NAME_TOO_LONG;
	}
	strcpy(device_name, name);
	return CUSTOM_BLE_OK;
}

custom_ble_result_t custom_ble_update_adv(bool use_raw, size_t adv_len, const uint8_t adv_data_raw[adv_len],
	size_t scan_rsp_len, const uint8_t scan_rsp_data_raw[scan_rsp_len]) {
	if ((adv_data_raw && adv_len > 31) || (scan_rsp_data_raw && scan_rsp_len > 31)) {
		return CUSTOM_BLE_TOO_LONG;
	}
	adv_update_t update = {
		.use_raw = use_raw,
		.adv_len = adv_len,
		.adv_data = adv_data_raw,
		.scan_rsp_len = scan_rsp_len,
		.scan_rsp_data = scan_rsp_data_raw,
	};
	int res = has_started ? comm_ble_host_call(update_advertising, &update) : update_advertising(&update);
	return res == 0 ? CUSTOM_BLE_OK : CUSTOM_BLE_ESP_ERROR;
}

void custom_ble_set_attr_write_handler(attr_write_cb_t callback) {
	attr_write_cb = callback;
}

custom_ble_result_t custom_ble_add_service(ble_uuid_any_t service_uuid, uint16_t chr_count,
	const ble_chr_definition_t chr[chr_count], service_handles_cb_t handles_cb) {
	if (!has_started) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	if (init_result != CUSTOM_BLE_OK) {
		return CUSTOM_BLE_INIT_FAILED;
	}
	if (custom_service_len >= service_capacity) {
		return CUSTOM_BLE_TOO_MANY_SERVICES;
	}
	uint32_t count = chr_count;
	for (int i = 0; i < chr_count; i++) {
		count += chr[i].descr_count;
	}
	if (count + custom_attr_len > chr_descr_capacity) {
		return CUSTOM_BLE_TOO_MANY_CHR_AND_DESCR;
	}
	// GAP and GATT are owned by the host. Deletion is by UUID in NimBLE.
	uint16_t uuid16 = ble_uuid_u16(&service_uuid.u);
	if (uuid16 == 0x1800 || uuid16 == 0x1801 || comm_ble_service_uuid_reserved(&service_uuid.u)) {
		return CUSTOM_BLE_ERROR;
	}
	// Duplicate service UUIDs cannot be removed safely.
	for (int i = 0; i < custom_service_len; i++) {
		if (ble_uuid_cmp(&service_uuid.u, &custom_services[i].uuid.u) == 0) {
			return CUSTOM_BLE_ERROR;
		}
	}
	service_instance_t *service = &custom_services[custom_service_len];
	service->uuid = service_uuid;
	service->attr_count = count;
	service->attr = calloc(count ? count : 1, sizeof(attr_instance_t));
	service->definition = calloc(2, sizeof(struct ble_gatt_svc_def));
	struct ble_gatt_chr_def *ble_chars = calloc(chr_count + 1, sizeof(*ble_chars));
	uint16_t *handles = calloc(count + 1, sizeof(uint16_t));
	if (service->definition) {
		service->definition[0].characteristics = ble_chars;
	}
	if (!service->attr || !service->definition || !ble_chars || !handles) {
		if (!service->definition) {
			free(ble_chars);
		}
		free(handles);
		free_service(service);
		return CUSTOM_BLE_ERROR;
	}
	service->definition[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
	service->definition[0].uuid = &service->uuid.u;
	uint16_t pos = 0;
	for (int i = 0; i < chr_count; i++) {
		attr_instance_t *attr = &service->attr[pos++];
		if (!init_attr(attr, chr[i].uuid, chr[i].value_max_len, chr[i].value_len, chr[i].value)) {
			goto fail;
		}
		attr->chr = &ble_chars[i];
		ble_chars[i].uuid = &attr->uuid.u;
		ble_chars[i].access_cb = attr_access_handler;
		ble_chars[i].arg = attr;
		ble_chars[i].flags = chr[i].property;
		ble_chars[i].val_handle = &attr->attr_handle;
		struct ble_gatt_dsc_def *descriptors = calloc(chr[i].descr_count + 1, sizeof(*descriptors));
		if (!descriptors) {
			goto fail;
		}
		ble_chars[i].descriptors = descriptors;
		int descr_pos = 0;
		bool has_cccd = false;
		for (int j = 0; j < chr[i].descr_count; j++) {
			const ble_desc_definition_t *desc = &chr[i].descriptors[j];
			attr_instance_t *descr = &service->attr[pos++];
			bool is_cccd = ble_uuid_u16(&desc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16
				&& (chr[i].property & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE));
			if (!init_attr(
					descr, desc->uuid, is_cccd ? 2 : desc->value_max_len, is_cccd ? 0 : desc->value_len, desc->value)) {
				goto fail;
			}
			descr->is_cccd = is_cccd;
			descr->chr = &ble_chars[i];
			if (is_cccd) {
				if (has_cccd) {
					goto fail;
				}
				has_cccd = true;
				// NimBLE owns the CCCD and inserts it immediately after the value.
				descr->value_len = 2;
				continue;
			}
			descriptors[descr_pos++] = (struct ble_gatt_dsc_def){
				.uuid = &descr->uuid.u,
				.att_flags = desc->perm,
				.access_cb = attr_access_handler,
				.arg = descr,
			};
		}
	}
	int res = comm_ble_host_call(register_service, service);
	if (res != 0) {
		if (init_result != CUSTOM_BLE_OK) {
			free(handles);
			return CUSTOM_BLE_INTERNAL_ERROR;
		}
		goto fail;
	}
	handles[0] = service->service_handle;
	for (int i = 0; i < service->attr_count; i++) {
		handles[i + 1] = service->attr[i].attr_handle;
	}
	if (handles_cb) {
		handles_cb(count + 1, handles);
	}
	free(handles);
	return CUSTOM_BLE_OK;
fail:
	free(handles);
	free_service(service);
	return CUSTOM_BLE_ERROR;
}

static int configure_hid(void *arg) {
	uint16_t appearance = *(uint16_t *)arg;
	int res = ble_svc_gap_device_appearance_set(appearance);
	if (res != 0) {
		return res;
	}
	service_instance_t *service = &custom_services[custom_service_len - 1];
	xSemaphoreTake(attr_mutex, portMAX_DELAY);
	service->hid = true;
	for (int i = 0; i < service->attr_count; i++) {
		service->attr[i].hid = true;
		if (ble_uuid_u16(&service->attr[i].uuid.u) == 0x2a4d) {
			service->attr[i].value_len = service->attr[i].value_max_len;
		}
	}
	xSemaphoreGive(attr_mutex);
	ble_hs_cfg.sm_bonding = 1;
	if (ble_gap_adv_active()) {
		ble_gap_adv_stop();
		start_advertising();
	}
	return 0;
}

custom_ble_result_t custom_ble_add_hid_service(const uint8_t *map, uint16_t map_len,
	const ble_hid_report_t *reports, uint16_t count, uint16_t appearance, service_handles_cb_t handles_cb) {
	if (!map || map_len == 0 || map_len > 512 || !reports || count == 0 || count > BLE_HID_REPORTS_MAX) {
		return CUSTOM_BLE_ERROR;
	}
	for (int i = 0; i < count; i++) {
		if (reports[i].type < 1 || reports[i].type > 3 || reports[i].size == 0 || reports[i].size > 512) {
			return CUSTOM_BLE_ERROR;
		}
		for (int j = 0; j < i; j++) {
			if ((reports[i].id == reports[j].id && reports[i].type == reports[j].type)
				|| ((reports[i].id == 0) != (reports[j].id == 0))) {
				return CUSTOM_BLE_ERROR;
			}
		}
	}
	ble_chr_definition_t *chars = calloc(count + 3, sizeof(*chars));
	ble_desc_definition_t *descs = calloc(count, sizeof(*descs));
	if (!chars || !descs) {
		free(chars);
		free(descs);
		return CUSTOM_BLE_ERROR;
	}
	uint8_t info[] = {0x11, 0x01, 0, 0x02}; // HID 1.11, normally connectable.
	uint8_t control = 1;
	uint8_t refs[BLE_HID_REPORTS_MAX][2];
	chars[0] = (ble_chr_definition_t){
		.uuid.u16      = BLE_UUID16_INIT(0x2a4a),
		.property = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
		.value_max_len = sizeof(info),
		.value_len = sizeof(info),
		.value = info
	};
	chars[1] = (ble_chr_definition_t){
		.uuid.u16      = BLE_UUID16_INIT(0x2a4b),
		.property = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
		.value_max_len = map_len,
		.value_len = map_len,
		.value = (uint8_t *)map
	};
	chars[2] = (ble_chr_definition_t){
		.uuid.u16      = BLE_UUID16_INIT(0x2a4c),
		.property = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
		.value_max_len = 1,
		.value_len = 1,
		.value = &control
	};
	for (int i = 0; i < count; i++) {
		refs[i][0] = reports[i].id;
		refs[i][1] = reports[i].type;
		descs[i] = (ble_desc_definition_t){
			.uuid.u16      = BLE_UUID16_INIT(0x2908),
			.perm = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
			.value_max_len = 2,
			.value_len = 2,
			.value = refs[i]
		};
		chars[i + 3] = (ble_chr_definition_t){
			.uuid.u16      = BLE_UUID16_INIT(0x2a4d),
			.property = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
			.value_max_len = reports[i].size,
			.descr_count = 1,
			.descriptors = &descs[i]
		};
		if (reports[i].type == 1) {
			chars[i + 3].property |= BLE_GATT_CHR_F_NOTIFY;
		} else {
			chars[i + 3].property |= BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC;
			if (reports[i].type == 2) {
				chars[i + 3].property |= BLE_GATT_CHR_F_WRITE_NO_RSP;
			}
		}
	}
	ble_uuid_any_t uuid = {.u16 = BLE_UUID16_INIT(0x1812)};
	custom_ble_result_t res = custom_ble_add_service(uuid, count + 3, chars, NULL);
	free(descs);
	free(chars);
	if (res != CUSTOM_BLE_OK) {
		return res;
	}
	service_instance_t *service = &custom_services[custom_service_len - 1];
	if (comm_ble_host_call(configure_hid, &appearance) != 0) {
		custom_ble_remove_service(service->service_handle);
		return CUSTOM_BLE_ESP_ERROR;
	}
	uint16_t handles[4 + 2 * BLE_HID_REPORTS_MAX];
	handles[0] = service->service_handle;
	for (int i = 0; i < service->attr_count; i++) {
		handles[i + 1] = service->attr[i].attr_handle;
	}
	if (handles_cb) {
		handles_cb(service->attr_count + 1, handles);
	}
	return CUSTOM_BLE_OK;
}

custom_ble_result_t custom_ble_remove_service(uint16_t service_handle) {
	if (!has_started) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	service_instance_t *service = get_service(service_handle);
	if (!service) {
		return CUSTOM_BLE_INVALID_HANDLE;
	}
	if (service != &custom_services[custom_service_len - 1]) {
		return CUSTOM_BLE_SERVICE_NOT_LAST;
	}
	return comm_ble_host_call(remove_service, service) == 0 ? CUSTOM_BLE_OK : CUSTOM_BLE_ESP_ERROR;
}

custom_ble_result_t custom_ble_get_attr_value(uint16_t attr_handle, uint16_t *length, const uint8_t **value) {
	if (!has_started) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	xSemaphoreTake(attr_mutex, portMAX_DELAY);
	attr_instance_t *attr = get_attr(attr_handle);
	if (attr) {
		*length = attr->value_len;
		*value = attr->value;
	}
	xSemaphoreGive(attr_mutex);
	return attr ? CUSTOM_BLE_OK : CUSTOM_BLE_INVALID_HANDLE;
}

custom_ble_result_t custom_ble_set_attr_value(uint16_t attr_handle, uint16_t length, const uint8_t value[length]) {
	if (!has_started) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	xSemaphoreTake(attr_mutex, portMAX_DELAY);
	attr_instance_t *attr = get_attr(attr_handle);
	if (!attr || length > attr->value_max_len || attr->is_cccd) {
		xSemaphoreGive(attr_mutex);
		return !attr ? CUSTOM_BLE_INVALID_HANDLE : CUSTOM_BLE_ERROR;
	}
	bool hid_report = attr->hid && ble_uuid_u16(&attr->uuid.u) == 0x2a4d;
	if (hid_report && length != attr->value_max_len) {
		xSemaphoreGive(attr_mutex);
		return CUSTOM_BLE_ERROR;
	}
	memcpy(attr->value, value, length);
	attr->value_len = length;
	bool notify = attr->notify_enabled;
	bool indicate = attr->indicate_enabled;
	if (hid_report) {
		for (int i = 0; i < custom_service_len; i++) {
			if (custom_services[i].hid && custom_services[i].attr[2].value[0] == 0) {
				notify = false;
			}
		}
	}
	xSemaphoreGive(attr_mutex);
	int res = 0;
	if (hid_report && conn_id != BLE_HS_CONN_HANDLE_NONE && notify) {
		struct ble_gap_conn_desc desc;
		if (ble_gap_conn_find(conn_id, &desc) != 0 || !desc.sec_state.encrypted
				|| length > ble_att_mtu(conn_id) - 3) {
			return CUSTOM_BLE_ESP_ERROR;
		}
	}
	if (conn_id != BLE_HS_CONN_HANDLE_NONE && notify) {
		struct os_mbuf *om = ble_hs_mbuf_from_flat(value, length);
		res = om ? ble_gatts_notify_custom(conn_id, attr_handle, om) : BLE_HS_ENOMEM;
	}
	if (res == 0 && conn_id != BLE_HS_CONN_HANDLE_NONE && indicate) {
		struct os_mbuf *om = ble_hs_mbuf_from_flat(value, length);
		res = om ? ble_gatts_indicate_custom(conn_id, attr_handle, om) : BLE_HS_ENOMEM;
	}
	return res == 0 ? CUSTOM_BLE_OK : CUSTOM_BLE_ESP_ERROR;
}

uint16_t custom_ble_service_count(void) {
	return has_started ? custom_service_len : 0;
}

uint16_t custom_ble_get_services(uint16_t capacity, uint16_t handles[capacity]) {
	uint16_t count = custom_ble_service_count();
	if (count > capacity) {
		count = capacity;
	}
	for (int i = 0; i < count; i++) {
		handles[i] = custom_services[i].service_handle;
	}
	return count;
}

int16_t custom_ble_attr_count(uint16_t service_handle) {
	service_instance_t *service = has_started ? get_service(service_handle) : NULL;
	return service ? service->attr_count : -1;
}

custom_ble_result_t custom_ble_get_attrs(
	uint16_t service_handle, uint16_t capacity, uint16_t handles[capacity], uint16_t *written_count) {
	if (!has_started) {
		return CUSTOM_BLE_NOT_STARTED;
	}
	service_instance_t *service = get_service(service_handle);
	if (!service) {
		return CUSTOM_BLE_INVALID_HANDLE;
	}
	*written_count = service->attr_count > capacity ? capacity : service->attr_count;
	for (int i = 0; i < *written_count; i++) {
		handles[i] = service->attr[i].attr_handle;
	}
	return CUSTOM_BLE_OK;
}

bool custom_ble_started(void) {
	return has_started;
}

static unsigned int client_capacity(void) {
	unsigned int capacity = CONFIG_BT_NIMBLE_MAX_CONNECTIONS - (backup.config.ble_mode != BLE_MODE_DISABLED || server_enabled);
	return capacity < BLE_CLIENT_CONNECTIONS_MAX ? capacity : BLE_CLIENT_CONNECTIONS_MAX;
}

void custom_ble_init(void) {
	server_enabled = backup.config.ble_mode_lbm == BLE_MODE_LBM_CLIENT_SERVER
		|| backup.config.ble_mode_lbm == BLE_MODE_LBM_SERVER;
	client_enabled = backup.config.ble_mode_lbm == BLE_MODE_LBM_CLIENT_SERVER
		|| backup.config.ble_mode_lbm == BLE_MODE_LBM_CLIENT;
	client_limit = client_capacity();
	for (unsigned int i = 0; i < BLE_CLIENT_CONNECTIONS_MAX; i++) {
		client_connections[i].conn = BLE_HS_CONN_HANDLE_NONE;
	}
	if (!server_enabled) {
		init_result = CUSTOM_BLE_NOT_STARTED;
		return;
	}
	service_capacity = backup.config.ble_service_capacity;
	chr_descr_capacity = backup.config.ble_chr_descr_capacity;
	if (service_capacity > 0) {
		custom_services = calloc(service_capacity, sizeof(*custom_services));
	}
	attr_mutex = xSemaphoreCreateMutex();
	init_result = (service_capacity == 0 || custom_services) && attr_mutex ? CUSTOM_BLE_OK : CUSTOM_BLE_ERROR;
	size_t len = strnlen((char *)backup.config.ble_name, sizeof(backup.config.ble_name));
	memcpy(device_name, (const char *)backup.config.ble_name, len);
	device_name[len] = '\0';
}

int custom_ble_reserve_resources(void) {
	if (!server_enabled || service_capacity == 0 || chr_descr_capacity == 0) {
		return 0;
	}
	// NimBLE sizes its CCCD pool when the host starts. Dynamic services added
	// later need one entry per notify/indicate characteristic for every link,
	// including outgoing client links, plus the server's configuration cache.
	// Count a worst-case definition without registering any dummy attributes.
	static const ble_uuid16_t reserve_uuid = BLE_UUID16_INIT(0xffff);
	struct ble_gatt_chr_def *chars = calloc(chr_descr_capacity + 1, sizeof(*chars));
	if (!chars) {
		return BLE_HS_ENOMEM;
	}
	for (uint16_t i = 0; i < chr_descr_capacity; i++) {
		chars[i].uuid = &reserve_uuid.u;
		chars[i].flags = BLE_GATT_CHR_F_NOTIFY;
		chars[i].access_cb = attr_access_handler;
	}
	struct ble_gatt_svc_def services[] = {
		{ .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &reserve_uuid.u, .characteristics = chars }, { 0 }
	};
	int res = ble_gatts_count_cfg(services);
	free(chars);
	return res;
}

static client_connection_t *client_find(uint16_t conn) {
	if (conn == BLE_HS_CONN_HANDLE_NONE) {
		return NULL;
	}
	for (unsigned int i = 0; i < BLE_CLIENT_CONNECTIONS_MAX; i++) {
		if (client_connections[i].conn == conn) {
			return &client_connections[i];
		}
	}
	return NULL;
}

unsigned int custom_ble_client_connections(uint16_t *handles, unsigned int capacity) {
	unsigned int count = 0;
	for (unsigned int i = 0; i < BLE_CLIENT_CONNECTIONS_MAX; i++) {
		uint16_t conn = client_connections[i].conn;
		if (conn != BLE_HS_CONN_HANDLE_NONE) {
			if (count < capacity && handles) {
				handles[count] = conn;
			}
			count++;
		}
	}
	return count;
}

unsigned int custom_ble_client_limit(void) {
	return client_limit;
}

static uint16_t client_callback_conn(uint16_t conn, void *arg) {
	return conn == BLE_HS_CONN_HANDLE_NONE ? (uint16_t)((uintptr_t)arg >> 8) : conn;
}

static void client_idle(uint16_t conn) {
	client_connection_t *client = client_find(conn);
	if (client) {
		client->busy = false;
	}
}

static void client_push(ble_client_event_t *event) {
	// Reserve room for procedure completions and connection state changes.
	// A slow script must not lose a write acknowledgement to notifications.
	bool data_event = event->type == BLE_CLIENT_SCAN || event->type == BLE_CLIENT_SERVICE
		|| event->type == BLE_CLIENT_CHR || event->type == BLE_CLIENT_DSC || event->type == BLE_CLIENT_NOTIFY;
	if ((data_event && uxQueueSpacesAvailable(client_events) <= client_capacity() + 1)
		|| xQueueSend(client_events, event, 0) != pdTRUE) {
		__atomic_add_fetch(&client_dropped, 1, __ATOMIC_RELAXED);
	}
}

static void client_connect_expired(struct ble_npl_event *event) {
	(void)event;
	if (client_connecting) {
		commands_printf("BLE connection timeout: restarting Bluetooth");
		ble_hs_sched_reset(BLE_HS_ETIMEOUT);
	}
}

static void client_restore_pair_policy(void) {
	if (client_pair_policy_pending) {
		ble_hs_cfg.sm_io_cap = client_saved_io;
		ble_hs_cfg.sm_bonding = client_saved_bond;
		ble_hs_cfg.sm_mitm = client_saved_mitm;
		client_pair_policy_pending = false;
	}
}

static int client_secure(client_connection_t *client, uint8_t io, bool bond) {
	if (io > BLE_HS_IO_KEYBOARD_DISPLAY) {
		return BLE_HS_EINVAL;
	}
	uint8_t io_cap = ble_hs_cfg.sm_io_cap;
	bool bonding = ble_hs_cfg.sm_bonding;
	bool mitm = ble_hs_cfg.sm_mitm;
	ble_hs_cfg.sm_io_cap = io;
	ble_hs_cfg.sm_bonding = bond;
	ble_hs_cfg.sm_mitm = io != BLE_HS_IO_NO_INPUT_OUTPUT;
	client->require_auth = ble_hs_cfg.sm_mitm;
	client->securing = true;
	int res = comm_ble_host_secure(client->conn);
	ble_hs_cfg.sm_io_cap = io_cap;
	ble_hs_cfg.sm_bonding = bonding;
	ble_hs_cfg.sm_mitm = mitm;
	if (res != 0) {
		client->securing = false;
	}
	return res;
}

static int client_gap_event(struct ble_gap_event *event, void *arg) {
	(void)arg;
	ble_client_event_t result = { .conn = BLE_HS_CONN_HANDLE_NONE };
	switch (event->type) {
		case BLE_GAP_EVENT_DISC:
			result.type = BLE_CLIENT_SCAN;
			result.address = event->disc.addr;
			result.rssi = event->disc.rssi;
			result.len = event->disc.length_data;
			if (result.len > sizeof(result.data)) {
				result.len = sizeof(result.data);
			}
			memcpy(result.data, event->disc.data, result.len);
			break;
		case BLE_GAP_EVENT_DISC_COMPLETE:
			result.type = BLE_CLIENT_SCAN_DONE;
			result.status = event->disc_complete.reason;
			break;
		case BLE_GAP_EVENT_CONNECT:
			client_connecting = false;
			ble_npl_callout_stop(&client_connect_timeout);
			result.type = BLE_CLIENT_CONNECT;
			result.status = event->connect.status;
			result.handle = BLE_HS_CONN_HANDLE_NONE;
			if (result.status == 0) {
				for (unsigned int i = 0; i < BLE_CLIENT_CONNECTIONS_MAX; i++) {
					if (client_connections[i].conn == BLE_HS_CONN_HANDLE_NONE) {
						client_connections[i].conn = event->connect.conn_handle;
						client_connections[i].busy = false;
						client_connections[i].securing = false;
						client_connections[i].require_auth = false;
						client_connections[i].passkey_action = BLE_SM_IOACT_NONE;
						result.handle = event->connect.conn_handle;
						result.conn = result.handle;
						break;
					}
				}
				if (result.handle == BLE_HS_CONN_HANDLE_NONE) {
					ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
					result.status = BLE_HS_ENOMEM;
				} else if (client_pair_on_connect) {
					client_push(&result);
					int res = client_secure(client_find(result.conn), client_pair_io, client_pair_bond);
					client_restore_pair_policy();
					if (res == 0) {
						return 0;
					}
					result.type = BLE_CLIENT_SECURITY;
					result.status = res;
				}
			}
			client_restore_pair_policy();
			break;
		case BLE_GAP_EVENT_DISCONNECT: {
			result.type = BLE_CLIENT_DISCONNECT;
			result.status = event->disconnect.reason;
			result.handle = event->disconnect.conn.conn_handle;
			result.conn = result.handle;
			client_connection_t *client = client_find(result.conn);
			if (client) {
				free(client->read_value);
				client->read_value = NULL;
				client->busy = false;
				client->securing = false;
				client->passkey_action = BLE_SM_IOACT_NONE;
				client->conn = BLE_HS_CONN_HANDLE_NONE;
			}
			break;
		}
		case BLE_GAP_EVENT_PASSKEY_ACTION: {
			result.conn = event->passkey.conn_handle;
			client_connection_t *client = client_find(result.conn);
			if (!client) {
				return 0;
			}
			client->securing = true;
			client->passkey_action = event->passkey.params.action;
			result.type = BLE_CLIENT_PASSKEY;
			result.handle = result.conn;
			result.status = client->passkey_action;
			if (client->passkey_action == BLE_SM_IOACT_NUMCMP) {
				result.passkey = event->passkey.params.numcmp;
			}
			break;
		}
		case BLE_GAP_EVENT_ENC_CHANGE: {
			result.type = BLE_CLIENT_SECURITY;
			result.conn = event->enc_change.conn_handle;
			result.handle = result.conn;
			result.status = event->enc_change.status;
			client_connection_t *client = client_find(result.conn);
			if (!client) {
				return 0;
			}
			bool require_auth = client->securing && client->require_auth;
			client->securing = false;
			client->passkey_action = BLE_SM_IOACT_NONE;
			struct ble_gap_conn_desc desc;
			int res = ble_gap_conn_find(result.conn, &desc);
			if (res == 0) {
				result.properties = desc.sec_state.encrypted | (desc.sec_state.authenticated << 1) |
					(desc.sec_state.bonded << 2);
				if (result.status == 0 && (!desc.sec_state.encrypted || (require_auth && !desc.sec_state.authenticated))) {
					result.status = require_auth ? BLE_HS_EAUTHEN : BLE_HS_EENCRYPT;
				}
			} else if (result.status == 0) {
				result.status = res;
			}
			break;
		}
		case BLE_GAP_EVENT_REPEAT_PAIRING:
			return BLE_GAP_REPEAT_PAIRING_IGNORE;
		case BLE_GAP_EVENT_NOTIFY_RX:
			result.type = BLE_CLIENT_NOTIFY;
			result.conn = event->notify_rx.conn_handle;
			result.handle = event->notify_rx.attr_handle;
			result.properties = event->notify_rx.indication;
			result.status = ble_hs_mbuf_to_flat(event->notify_rx.om, result.data, sizeof(result.data), &result.len);
			break;
		default:
			return 0;
	}
	client_push(&result);
	return 0;
}

static void client_done(uint16_t conn, int status, ble_client_op_t op) {
	client_idle(conn);
	ble_client_event_t event = {
		.type = BLE_CLIENT_DONE, .conn = conn, .status = status == BLE_HS_EDONE ? 0 : status, .handle = op
	};
	client_push(&event);
}

static int client_service(
	uint16_t conn, const struct ble_gatt_error *error, const struct ble_gatt_svc *svc, void *arg) {
	conn = client_callback_conn(conn, arg);
	if (error->status != 0) {
		client_done(conn, error->status, BLE_CLIENT_OP_SERVICES);
	} else {
		ble_client_event_t event = { .type = BLE_CLIENT_SERVICE,
			.conn = conn,
			.handle = svc->start_handle,
			.end = svc->end_handle,
			.uuid = svc->uuid };
		client_push(&event);
	}
	return 0;
}

static int client_characteristic(
	uint16_t conn, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
	conn = client_callback_conn(conn, arg);
	if (error->status != 0) {
		client_done(conn, error->status, BLE_CLIENT_OP_CHRS);
	} else {
		ble_client_event_t event = { .type = BLE_CLIENT_CHR,
			.conn = conn,
			.handle = chr->val_handle,
			.end = chr->def_handle,
			.properties = chr->properties,
			.uuid = chr->uuid };
		client_push(&event);
	}
	return 0;
}

static int client_descriptor(
	uint16_t conn, const struct ble_gatt_error *error, uint16_t chr, const struct ble_gatt_dsc *dsc, void *arg) {
	(void)chr;
	conn = client_callback_conn(conn, arg);
	if (error->status != 0) {
		client_done(conn, error->status, BLE_CLIENT_OP_DSCS);
	} else {
		ble_client_event_t event = { .type = BLE_CLIENT_DSC, .conn = conn, .handle = dsc->handle, .uuid = dsc->uuid };
		client_push(&event);
	}
	return 0;
}

static int client_value(uint16_t conn, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
	conn = client_callback_conn(conn, arg);
	ble_client_op_t op = (ble_client_op_t)((uintptr_t)arg & 0xff);
	ble_client_event_t event = { .type = op == BLE_CLIENT_OP_READ ? BLE_CLIENT_READ : BLE_CLIENT_WRITE,
		.conn = conn,
		.status = error->status,
		.handle = error->att_handle };
	if (attr) {
		event.handle = attr->handle;
		if (op == BLE_CLIENT_OP_READ && error->status == 0) {
			event.status = ble_hs_mbuf_to_flat(attr->om, event.data, sizeof(event.data), &event.len);
		}
	}
	client_idle(conn);
	client_push(&event);
	return 0;
}

static int client_read_long(uint16_t conn, const struct ble_gatt_error *error, struct ble_gatt_attr *attr, void *arg) {
	conn = client_callback_conn(conn, arg);
	client_connection_t *client = client_find(conn);
	if (!client || !client->read_value) {
		return BLE_HS_ENOTCONN;
	}
	int status = error->status;
	if (status == 0 && attr) {
		uint16_t len = 0;
		if (attr->offset != client->read_len) {
			status = BLE_HS_EBADDATA;
		} else {
			status = ble_hs_mbuf_to_flat(attr->om, client->read_value + client->read_len,
				BLE_CLIENT_ATTR_MAX - client->read_len, &len);
			client->read_len += len;
		}
		if (status == 0) {
			return 0;
		}
	}
	ble_client_event_t event = {
		.type = BLE_CLIENT_READ,
		.conn = conn,
		.handle = client->read_handle,
		.status = status == BLE_HS_EDONE ? 0 : status
	};
	if (event.status == 0) {
		event.len = client->read_len;
		memcpy(event.data, client->read_value, event.len);
	}
	free(client->read_value);
	client->read_value = NULL;
	client_idle(conn);
	client_push(&event);
	return status == BLE_HS_EDONE ? 0 : status;
}

static int client_mtu(uint16_t conn, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
	conn = client_callback_conn(conn, arg);
	ble_client_event_t event = { .type = BLE_CLIENT_MTU, .conn = conn, .handle = mtu, .status = error->status };
	client_idle(conn);
	client_push(&event);
	return 0;
}

static int client_request_on_host(void *arg) {
	ble_client_request_t *r = arg;
	if (r->op == BLE_CLIENT_OP_LIMIT) {
		if (r->duration_ms < 1 || r->duration_ms > client_capacity()) {
			return BLE_HS_EINVAL;
		}
		if (custom_ble_client_connections(NULL, 0) + client_connecting > r->duration_ms) {
			return BLE_HS_EBUSY;
		}
		client_limit = r->duration_ms;
		return 0;
	}
	if (r->op == BLE_CLIENT_OP_SCAN_STOP) {
		return ble_gap_disc_cancel();
	}
	if (r->op == BLE_CLIENT_OP_DISCONNECT) {
		if (r->conn != BLE_HS_CONN_HANDLE_NONE) {
			return client_find(r->conn) ? ble_gap_terminate(r->conn, BLE_ERR_REM_USER_CONN_TERM) : BLE_HS_ENOTCONN;
		}
		int res = BLE_HS_ENOTCONN;
		if (client_connecting) {
			res = ble_gap_conn_cancel();
		}
		for (unsigned int i = 0; i < BLE_CLIENT_CONNECTIONS_MAX; i++) {
			uint16_t conn = client_connections[i].conn;
			if (conn != BLE_HS_CONN_HANDLE_NONE) {
				int status = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
				if (res == BLE_HS_ENOTCONN || status != 0) {
					res = status;
				}
			}
		}
		return res;
	}
	uint16_t conn = r->conn == BLE_HS_CONN_HANDLE_NONE ? custom_ble_client_conn_handle() : r->conn;
	client_connection_t *client = client_find(conn);
	if (r->op != BLE_CLIENT_OP_SCAN && r->op != BLE_CLIENT_OP_CONNECT && !client) {
		return BLE_HS_ENOTCONN;
	}
	if (r->op == BLE_CLIENT_OP_RSSI) {
		return ble_gap_conn_rssi(conn, &r->rssi);
	}
	if (r->op == BLE_CLIENT_OP_CONNECT) {
		if (client_connecting) {
			return BLE_HS_EBUSY;
		}
		struct ble_gap_conn_desc existing;
		if (ble_gap_conn_find_by_addr(&r->address, &existing) == 0) {
			return BLE_HS_EDONE;
		}
		if (custom_ble_client_connections(NULL, 0) >= client_limit) {
			return BLE_HS_ENOMEM;
		}
	}
	uint8_t client_addr_type;
	int addr_res = ble_hs_id_infer_auto(0, &client_addr_type);
	if (addr_res != 0) {
		return addr_res;
	}
	if (!client_events) {
		unsigned int count = 8 + 2 * client_capacity();
		client_events = xQueueCreate(count < 16 ? 16 : count, sizeof(ble_client_event_t));
		if (!client_events) {
			return BLE_HS_ENOMEM;
		}
		ble_npl_callout_init(&client_connect_timeout, nimble_port_get_dflt_eventq(), client_connect_expired, NULL);
	}
	if (r->op == BLE_CLIENT_OP_SCAN) {
		struct ble_gap_disc_params params = { .passive = 0, .filter_duplicates = 1, .itvl = 160, .window = 80 };
#if CONFIG_IDF_TARGET_ESP32P4
		if (comm_wifi_get_mode() == WIFI_MODE_ACCESS_POINT) {
			params.window = 16;
		}
#endif
		return ble_gap_disc(client_addr_type, r->duration_ms, &params, client_gap_event, NULL);
	}
	if (r->op == BLE_CLIENT_OP_CONNECT) {
		if (ble_gap_disc_active()) {
			ble_gap_disc_cancel();
		}
		client_pair_on_connect = r->security;
		client_pair_io = r->io_cap;
		client_pair_bond = r->bond;
		if (r->security) {
			client_saved_io = ble_hs_cfg.sm_io_cap;
			client_saved_bond = ble_hs_cfg.sm_bonding;
			client_saved_mitm = ble_hs_cfg.sm_mitm;
			client_pair_policy_pending = true;
			ble_hs_cfg.sm_io_cap = r->io_cap;
			ble_hs_cfg.sm_bonding = r->bond;
			ble_hs_cfg.sm_mitm = r->io_cap != BLE_HS_IO_NO_INPUT_OUTPUT;
		}
		const struct ble_gap_conn_params *params = NULL;
#if CONFIG_IDF_TARGET_ESP32
		const struct ble_gap_conn_params esp32_params = {
			.scan_itvl = 160, .scan_window = 80,
			.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,
			.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,
			.latency = BLE_GAP_INITIAL_CONN_LATENCY,
			.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,
			.min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
			.max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
		};
		params = &esp32_params;
#endif
		int res = ble_gap_connect(client_addr_type, &r->address, r->duration_ms, params, client_gap_event, NULL);
		client_connecting = res == 0;
		if (res == 0 && ble_npl_callout_reset(&client_connect_timeout, ble_npl_time_ms_to_ticks32(r->duration_ms + 1500)) != 0) {
			ble_hs_sched_reset(BLE_HS_ENOMEM);
		}
		if (res != 0) {
			client_restore_pair_policy();
		}
		return res;
	}
	if (r->op == BLE_CLIENT_OP_PASSKEY) {
		struct ble_sm_io io = { .action = client->passkey_action };
		if (io.action == BLE_SM_IOACT_NUMCMP && r->action == BLE_SM_IOACT_NUMCMP && r->passkey <= 1) {
			io.numcmp_accept = r->passkey;
		} else if ((io.action == BLE_SM_IOACT_INPUT || io.action == BLE_SM_IOACT_DISP) &&
				r->action == BLE_SM_IOACT_INPUT && r->passkey <= 999999) {
			io.passkey = r->passkey;
		} else {
			return BLE_HS_EINVAL;
		}
		int res = ble_sm_inject_io(conn, &io);
		if (res == 0) {
			client->passkey_action = BLE_SM_IOACT_NONE;
		}
		return res;
	}
	if (client->busy || client->securing) {
		return BLE_HS_EBUSY;
	}
	if (r->op == BLE_CLIENT_OP_SECURITY) {
		return client_secure(client, r->io_cap, r->bond);
	}
	if (r->op == BLE_CLIENT_OP_FORGET) {
		struct ble_gap_conn_desc desc;
		int res = ble_gap_conn_find(conn, &desc);
		if (res == 0) {
			res = ble_store_util_delete_peer(&desc.peer_id_addr);
		}
		if (res == 0) {
			res = ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
		}
		return res;
	}
	client->busy = true;
	void *callback_arg = (void *)(((uintptr_t)conn << 8) | r->op);
	int res;
	switch (r->op) {
		case BLE_CLIENT_OP_MTU:
			res = ble_gattc_exchange_mtu(conn, client_mtu, callback_arg);
			break;
		case BLE_CLIENT_OP_SERVICES:
			res = ble_gattc_disc_all_svcs(conn, client_service, callback_arg);
			break;
		case BLE_CLIENT_OP_CHRS:
			res = ble_gattc_disc_all_chrs(conn, r->start, r->end, client_characteristic, callback_arg);
			break;
		case BLE_CLIENT_OP_DSCS:
			res = ble_gattc_disc_all_dscs(conn, r->start, r->end, client_descriptor, callback_arg);
			break;
		case BLE_CLIENT_OP_READ:
			res = ble_gattc_read(conn, r->start, client_value, callback_arg);
			break;
		case BLE_CLIENT_OP_READ_LONG:
			client->read_value = malloc(BLE_CLIENT_ATTR_MAX);
			client->read_len = 0;
			client->read_handle = r->start;
			if (!client->read_value) {
				res = BLE_HS_ENOMEM;
			} else {
				res = ble_gattc_read_long(conn, r->start, 0, client_read_long, callback_arg);
				if (res != 0) {
					free(client->read_value);
					client->read_value = NULL;
				}
			}
			break;
		case BLE_CLIENT_OP_WRITE:
		case BLE_CLIENT_OP_WRITE_NR:
			if (r->len > ble_att_mtu(conn) - 3) {
				res = BLE_HS_EMSGSIZE;
			} else if (r->op == BLE_CLIENT_OP_WRITE) {
				res = ble_gattc_write_flat(conn, r->start, r->data, r->len, client_value, callback_arg);
			} else {
				res = ble_gattc_write_no_rsp_flat(conn, r->start, r->data, r->len);
				client->busy = false;
			}
			break;
		default:
			res = BLE_HS_EINVAL;
			break;
	}
	if (res != 0) {
		client->busy = false;
	}
	return res;
}

int custom_ble_client_request(ble_client_request_t *request) {
	if (!client_enabled) {
		return BLE_HS_ENOTSUP;
	}
	if (!comm_ble_host_ready()) {
		return BLE_HS_ENOTSYNCED;
	}
	return comm_ble_host_call(client_request_on_host, request);
}

bool custom_ble_client_event(ble_client_event_t *event, bool consume) {
	if (!client_events) {
		return false;
	}
	return (consume ? xQueueReceive(client_events, event, 0) : xQueuePeek(client_events, event, 0)) == pdTRUE;
}

uint32_t custom_ble_client_dropped(void) {
	return __atomic_load_n(&client_dropped, __ATOMIC_RELAXED);
}

uint16_t custom_ble_client_conn_handle(void) {
	uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
	custom_ble_client_connections(&conn, 1);
	return conn;
}
#else
void custom_ble_init(void) {
}

bool custom_ble_started(void) {
	return false;
}
#endif
