/*
	NimBLE implementation of the custom BLE GATT API.

	This implements the same public API as custom_ble.c so that the LispBM
	ble-* extensions work with either BLE host stack.

	NimBLE has a declarative GATT API: services are registered as a
	ble_gatt_svc_def[] array. To match the Bluedroid dynamic add/remove
	semantics we keep an internal table of services and attributes and
	rebuild the entire NimBLE service registration on every mutation
	(ble_gatts_reset -> ble_gatts_add_svcs -> ble_gatts_start). Handles are
	captured in declaration order via ble_hs_cfg.gatts_register_cb.

	Mutating GATT requires no active connection, so add/remove terminates the
	current peer first if one is connected.
*/

#include "custom_ble.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "conf_general.h"
#include "main.h"

#ifndef HW_BLE_PWR_LVL
#if CONFIG_IDF_TARGET_ESP32C6
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P9
#else
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P18
#endif
#endif
#define ESP_PWR_LVL HW_BLE_PWR_LVL

void ble_store_config_init(void);

/* ---------- internal state (mirrors the Bluedroid version) ---------- */

typedef uint8_t custom_ble_id_t;

typedef enum {
	CUSTOM_BLE_TYPE_CHR,
	CUSTOM_BLE_TYPE_DESCR,
} custom_ble_attr_type_t;

typedef struct {
	custom_ble_id_t service_index;
	uint16_t handle; // NimBLE handle (val_handle for chr, dsc handle for descr)
	esp_bt_uuid_t uuid;
	custom_ble_attr_type_t type;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t prop; // chr only
	uint16_t value_max_len;
	uint16_t value_len;
	uint8_t *value; // owned, heap-allocated
	bool initialized;
} attr_instance_t;

typedef struct {
	uint16_t service_handle;
	esp_bt_uuid_t uuid;
	bool initialized;
} service_instance_t;

static bool has_started                = false;
static custom_ble_result_t init_result = CUSTOM_BLE_INIT_FAILED;
static uint16_t service_capacity       = 0;
static uint16_t chr_descr_capacity     = 0;

static char device_name[CUSTOM_BLE_MAX_NAME_LEN + 1];
static attr_write_cb_t attr_write_cb = NULL;

static size_t custom_service_len           = 0;
static service_instance_t *custom_services = NULL;
static size_t custom_attr_len              = 0;
static attr_instance_t *custom_attr        = NULL;

static bool use_custom_adv_data          = false;
static size_t ble_adv_data_raw_len       = 0;
static uint8_t ble_adv_data_raw[31]      = {0};
static size_t ble_scan_rsp_data_raw_len  = 0;
static uint8_t ble_scan_rsp_data_raw[31] = {0};

static bool is_connected         = false;
static uint16_t conn_handle      = 0;
static uint16_t ble_current_mtu  = 23;
static uint8_t own_addr_type     = 0;
static volatile bool host_synced = false;

static SemaphoreHandle_t disc_sem = NULL; // signaled on DISCONNECT

/* ---------- ble_adv_params (legacy global from custom_ble.h) ---------- */

esp_ble_adv_params_t ble_adv_params = {
	.adv_int_min       = 0x20,
	.adv_int_max       = 0x40,
	.adv_type          = ADV_TYPE_IND,
	.own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
	.channel_map       = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ---------- helpers ---------- */

static void apply_tx_power(void) {
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL);
}

/* Convert Bluedroid-shaped UUID to a NimBLE ble_uuid_any_t.
   Bluedroid 128-bit UUIDs are stored little-endian; NimBLE ble_uuid128_t
   .value is also LE, so the 128-bit case is a straight memcpy. */
static bool bd_uuid_to_nimble(const esp_bt_uuid_t *in, ble_uuid_any_t *out) {
	switch (in->len) {
		case ESP_UUID_LEN_16:
			out->u.type    = BLE_UUID_TYPE_16;
			out->u16.value = in->uuid.uuid16;
			return true;
		case ESP_UUID_LEN_32:
			out->u.type    = BLE_UUID_TYPE_32;
			out->u32.value = in->uuid.uuid32;
			return true;
		case ESP_UUID_LEN_128:
			out->u.type = BLE_UUID_TYPE_128;
			memcpy(out->u128.value, in->uuid.uuid128, ESP_UUID_LEN_128);
			return true;
		default:
			return false;
	}
}

static ble_gatt_chr_flags chr_flags_from(
	esp_gatt_char_prop_t prop, esp_gatt_perm_t perm
) {
	ble_gatt_chr_flags f = 0;
	if (prop & ESP_GATT_CHAR_PROP_BIT_READ)
		f |= BLE_GATT_CHR_F_READ;
	if (prop & ESP_GATT_CHAR_PROP_BIT_WRITE)
		f |= BLE_GATT_CHR_F_WRITE;
	if (prop & ESP_GATT_CHAR_PROP_BIT_WRITE_NR)
		f |= BLE_GATT_CHR_F_WRITE_NO_RSP;
	if (prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
		f |= BLE_GATT_CHR_F_NOTIFY;
	if (prop & ESP_GATT_CHAR_PROP_BIT_INDICATE)
		f |= BLE_GATT_CHR_F_INDICATE;
	if (prop & ESP_GATT_CHAR_PROP_BIT_BROADCAST)
		f |= BLE_GATT_CHR_F_BROADCAST;

	if (perm & ESP_GATT_PERM_READ_ENCRYPTED)
		f |= BLE_GATT_CHR_F_READ_ENC;
	if (perm & ESP_GATT_PERM_READ_ENC_MITM)
		f |= BLE_GATT_CHR_F_READ_AUTHEN;
	if (perm & ESP_GATT_PERM_WRITE_ENCRYPTED)
		f |= BLE_GATT_CHR_F_WRITE_ENC;
	if (perm & ESP_GATT_PERM_WRITE_ENC_MITM)
		f |= BLE_GATT_CHR_F_WRITE_AUTHEN;
	if (perm & ESP_GATT_PERM_WRITE_SIGNED)
		f |= BLE_GATT_CHR_F_AUTH_SIGN_WRITE;
	return f;
}

static uint8_t dsc_flags_from(esp_gatt_perm_t perm) {
	uint8_t f = 0;
	if (perm & ESP_GATT_PERM_READ)
		f |= BLE_ATT_F_READ;
	if (perm & ESP_GATT_PERM_WRITE)
		f |= BLE_ATT_F_WRITE;
	if (perm & ESP_GATT_PERM_READ_ENCRYPTED)
		f |= BLE_ATT_F_READ_ENC;
	if (perm & ESP_GATT_PERM_READ_ENC_MITM)
		f |= BLE_ATT_F_READ_AUTHEN;
	if (perm & ESP_GATT_PERM_WRITE_ENCRYPTED)
		f |= BLE_ATT_F_WRITE_ENC;
	if (perm & ESP_GATT_PERM_WRITE_ENC_MITM)
		f |= BLE_ATT_F_WRITE_AUTHEN;
	return f;
}

static int16_t get_attr_index_by_handle(uint16_t handle) {
	for (size_t i = 0; i < custom_attr_len; i++) {
		if (custom_attr[i].initialized && custom_attr[i].handle == handle) {
			return (int16_t)i;
		}
	}
	return -1;
}

static bool get_service_index(
	uint16_t service_handle, custom_ble_id_t *idx_out
) {
	for (size_t i = 0; i < custom_service_len; i++) {
		if (custom_services[i].initialized
			&& custom_services[i].service_handle == service_handle) {
			*idx_out = (custom_ble_id_t)i;
			return true;
		}
	}
	return false;
}

/* ---------- NimBLE service-table builder ----------

   We build everything from custom_services[] / custom_attr[] each rebuild.
   `initialized` only means NimBLE has reported a handle for the entry, so
   staged entries must still be included while a rebuild is in progress.
   All the heap allocations below are owned by `built_*` and freed at the
   start of the next rebuild (or in custom_ble_init failure paths).
*/

static struct ble_gatt_svc_def *built_svcs =
	NULL; // size custom_service_len + 1 (terminator)
static struct ble_gatt_chr_def *built_chrs = NULL; // pool, sliced per-service
static struct ble_gatt_dsc_def *built_dscs = NULL; // pool, sliced per-chr
static ble_uuid_any_t *built_uuids = NULL; // pool: 1 svc + chr_descr_capacity
static uint16_t *built_val_handles = NULL; // chr val_handle storage
static uint16_t built_chr_count    = 0;
static uint16_t built_dsc_count    = 0;

static void free_built_tables(void) {
	free(built_svcs);
	built_svcs = NULL;
	free(built_chrs);
	built_chrs = NULL;
	free(built_dscs);
	built_dscs = NULL;
	free(built_uuids);
	built_uuids = NULL;
	free(built_val_handles);
	built_val_handles = NULL;
	built_chr_count   = 0;
	built_dsc_count   = 0;
}

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
);

/* Build the entire ble_gatt_svc_def[] from custom_services / custom_attr.
   Returns 0 on success. */
static int build_service_tables(void) {
	free_built_tables();

	if (custom_service_len == 0) {
		// One terminator entry, no services.
		built_svcs = calloc(1, sizeof(struct ble_gatt_svc_def));
		return built_svcs ? 0 : -1;
	}

	// Count chrs and dscs (descriptors).
	uint16_t total_chrs = 0;
	uint16_t total_dscs = 0;
	for (size_t i = 0; i < custom_attr_len; i++) {
		if (custom_attr[i].type == CUSTOM_BLE_TYPE_CHR)
			total_chrs++;
		else
			total_dscs++;
	}

	// One uuid per service + one per attr (chr or dsc).
	size_t total_uuids = custom_service_len + total_chrs + total_dscs;

	built_svcs =
		calloc(custom_service_len + 1, sizeof(struct ble_gatt_svc_def));
	built_chrs = calloc(
		total_chrs + custom_service_len, sizeof(struct ble_gatt_chr_def)
	); // +1 terminator per svc
	built_dscs        = (total_dscs > 0)
			   ? calloc(
              total_dscs + total_chrs, sizeof(struct ble_gatt_dsc_def)
          ) // +1 terminator per chr that has dscs
			   : NULL;
	built_uuids       = calloc(total_uuids, sizeof(ble_uuid_any_t));
	built_val_handles = calloc(total_chrs ? total_chrs : 1, sizeof(uint16_t));

	if (!built_svcs || !built_chrs || !built_uuids || !built_val_handles
		|| (total_dscs > 0 && !built_dscs)) {
		free_built_tables();
		return -1;
	}

	built_chr_count = total_chrs;
	built_dsc_count = total_dscs;

	size_t uuid_pool_idx    = 0;
	size_t chr_pool_idx     = 0;
	size_t dsc_pool_idx     = 0;
	uint16_t val_handle_idx = 0;

	for (size_t s = 0; s < custom_service_len; s++) {
		// Convert service uuid.
		ble_uuid_any_t *svc_uuid = &built_uuids[uuid_pool_idx++];
		if (!bd_uuid_to_nimble(&custom_services[s].uuid, svc_uuid)) {
			free_built_tables();
			return -1;
		}

		// Slice off this service's chr range.
		struct ble_gatt_chr_def *svc_chrs = &built_chrs[chr_pool_idx];
		size_t svc_chr_count              = 0;

		for (size_t a = 0; a < custom_attr_len; a++) {
			if (custom_attr[a].service_index != s)
				continue;
			if (custom_attr[a].type != CUSTOM_BLE_TYPE_CHR)
				continue;

			ble_uuid_any_t *chr_uuid = &built_uuids[uuid_pool_idx++];
			if (!bd_uuid_to_nimble(&custom_attr[a].uuid, chr_uuid)) {
				free_built_tables();
				return -1;
			}

			// Slice off this chr's dsc range.
			struct ble_gatt_dsc_def *chr_dscs = NULL;
			size_t chr_dsc_count              = 0;
			if (built_dscs) {
				chr_dscs = &built_dscs[dsc_pool_idx];
				for (size_t d = a + 1; d < custom_attr_len; d++) {
					if (custom_attr[d].service_index != s)
						continue;
					if (custom_attr[d].type == CUSTOM_BLE_TYPE_CHR)
						break;
					// Descriptor of this chr.
					ble_uuid_any_t *dsc_uuid = &built_uuids[uuid_pool_idx++];
					if (!bd_uuid_to_nimble(&custom_attr[d].uuid, dsc_uuid)) {
						free_built_tables();
						return -1;
					}
					chr_dscs[chr_dsc_count++] = (struct ble_gatt_dsc_def){
						.uuid      = &dsc_uuid->u,
						.att_flags = dsc_flags_from(custom_attr[d].perm),
						.access_cb = gatt_access_cb,
						.arg       = &custom_attr[d],
					};
				}
				// Terminator.
				chr_dscs[chr_dsc_count] = (struct ble_gatt_dsc_def){0};
				dsc_pool_idx           += chr_dsc_count + 1;
			}

			svc_chrs[svc_chr_count++] = (struct ble_gatt_chr_def){
				.uuid      = &chr_uuid->u,
				.access_cb = gatt_access_cb,
				.arg       = &custom_attr[a],
				.flags =
					chr_flags_from(custom_attr[a].prop, custom_attr[a].perm),
				.val_handle  = &built_val_handles[val_handle_idx++],
				.descriptors = (chr_dsc_count > 0) ? chr_dscs : NULL,
			};
		}

		// Terminator chr for this service.
		svc_chrs[svc_chr_count] = (struct ble_gatt_chr_def){0};
		chr_pool_idx           += svc_chr_count + 1;

		built_svcs[s] = (struct ble_gatt_svc_def){
			.type            = BLE_GATT_SVC_TYPE_PRIMARY,
			.uuid            = &svc_uuid->u,
			.characteristics = svc_chrs,
		};
	}

	// Service array terminator.
	built_svcs[custom_service_len] = (struct ble_gatt_svc_def){0};
	return 0;
}

/* ---------- handle-capture from gatts_register_cb ----------

   NimBLE invokes ble_hs_cfg.gatts_register_cb once for each service / chr /
   dsc as ble_gatts_start() is processing. The events fire in declaration
   order, but NimBLE auto-inserts CCCDs for chars with NOTIFY/INDICATE
   flags; those come through as DSC events with UUID 0x2902 and must be
   skipped (they aren't in our user-defined attr table).
*/

static size_t reg_cur_service      = 0;
static size_t reg_cur_chr_attr_idx = 0;
static size_t reg_cur_dsc_attr_idx = 0;

static void on_gatts_register(struct ble_gatt_register_ctxt *ctxt, void *arg) {
	(void)arg;
	switch (ctxt->op) {
		case BLE_GATT_REGISTER_OP_SVC: {
			if (reg_cur_service < custom_service_len) {
				custom_services[reg_cur_service].service_handle =
					ctxt->svc.handle;
				custom_services[reg_cur_service].initialized = true;
				reg_cur_service++;

				// Reset the attr scan cursor to the first attr of this service.
				// We advance reg_cur_chr_attr_idx / reg_cur_dsc_attr_idx as
				// CHR/DSC events fire in declaration order.
			}
			break;
		}
		case BLE_GATT_REGISTER_OP_CHR: {
			// Advance to next CHR attr that belongs to a registered service.
			while (reg_cur_chr_attr_idx < custom_attr_len) {
				attr_instance_t *a = &custom_attr[reg_cur_chr_attr_idx];
				if (a->type == CUSTOM_BLE_TYPE_CHR) {
					a->handle      = ctxt->chr.val_handle;
					a->initialized = true;
					reg_cur_chr_attr_idx++;
					break;
				}
				reg_cur_chr_attr_idx++;
			}
			break;
		}
		case BLE_GATT_REGISTER_OP_DSC: {
			// Skip auto-CCCDs (UUID 0x2902).
			if (ctxt->dsc.dsc_def->uuid->type == BLE_UUID_TYPE_16
				&& ((const ble_uuid16_t *)ctxt->dsc.dsc_def->uuid)->value
					== 0x2902) {
				break;
			}
			// Advance to next DSC attr.
			while (reg_cur_dsc_attr_idx < custom_attr_len) {
				attr_instance_t *a = &custom_attr[reg_cur_dsc_attr_idx];
				if (a->type == CUSTOM_BLE_TYPE_DESCR) {
					a->handle      = ctxt->dsc.handle;
					a->initialized = true;
					reg_cur_dsc_attr_idx++;
					break;
				}
				reg_cur_dsc_attr_idx++;
			}
			break;
		}
		default:
			break;
	}
}

/* ---------- advertising ---------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* Re-arm advertising with the global gap cb. NimBLE doesn't have a global
   gap cb concept; we install per-adv-start callbacks. */
static void start_advertising_with_cb(void) {
	if (!host_synced)
		return;

	if (use_custom_adv_data) {
		ble_gap_adv_set_data(ble_adv_data_raw, ble_adv_data_raw_len);
		if (ble_scan_rsp_data_raw_len > 0) {
			ble_gap_adv_rsp_set_data(
				ble_scan_rsp_data_raw, ble_scan_rsp_data_raw_len
			);
		}
	} else {
		struct ble_hs_adv_fields adv_fields = {0};
		adv_fields.flags    = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
		adv_fields.name     = (uint8_t *)device_name;
		adv_fields.name_len = strlen(device_name);
		adv_fields.name_is_complete = 1;
		ble_gap_adv_set_fields(&adv_fields);
	}

	struct ble_gap_adv_params params = {0};
	params.conn_mode                 = BLE_GAP_CONN_MODE_UND;
	params.disc_mode                 = BLE_GAP_DISC_MODE_GEN;
	params.itvl_min                  = ble_adv_params.adv_int_min;
	params.itvl_max                  = ble_adv_params.adv_int_max;
	ble_gap_adv_start(
		own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event_cb, NULL
	);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
	(void)arg;
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status == 0) {
				conn_handle  = event->connect.conn_handle;
				is_connected = true;
				LED_BLUE_ON();
				apply_tx_power();
			} else {
				start_advertising_with_cb();
			}
			return 0;

		case BLE_GAP_EVENT_DISCONNECT:
			is_connected = false;
			LED_BLUE_OFF();
			if (disc_sem)
				xSemaphoreGive(disc_sem);
			start_advertising_with_cb();
			return 0;

		case BLE_GAP_EVENT_MTU:
			ble_current_mtu = event->mtu.value;
			return 0;

		case BLE_GAP_EVENT_REPEAT_PAIRING: {
			struct ble_gap_conn_desc desc;
			if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc)
				== 0) {
				ble_store_util_delete_peer(&desc.peer_id_addr);
			}
			return BLE_GAP_REPEAT_PAIRING_RETRY;
		}

		default:
			return 0;
	}
}

/* ---------- gatt access (single cb for all chrs + dscs) ---------- */

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
) {
	(void)conn_h;
	attr_instance_t *a = (attr_instance_t *)arg;
	if (a == NULL)
		return BLE_ATT_ERR_UNLIKELY;

	switch (ctxt->op) {
		case BLE_GATT_ACCESS_OP_READ_CHR:
		case BLE_GATT_ACCESS_OP_READ_DSC: {
			if (a->value == NULL || a->value_len == 0)
				return 0;
			int rc = os_mbuf_append(ctxt->om, a->value, a->value_len);
			return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		case BLE_GATT_ACCESS_OP_WRITE_CHR:
		case BLE_GATT_ACCESS_OP_WRITE_DSC: {
			if (a->value == NULL || a->value_max_len == 0) {
				return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
			}
			uint16_t in_len = OS_MBUF_PKTLEN(ctxt->om);
			if (in_len > a->value_max_len)
				in_len = a->value_max_len;

			uint16_t out_len = 0;
			int rc = ble_hs_mbuf_to_flat(ctxt->om, a->value, in_len, &out_len);
			if (rc != 0)
				return BLE_ATT_ERR_UNLIKELY;
			a->value_len = out_len;

			if (attr_write_cb != NULL) {
				attr_write_cb(a->handle, out_len, a->value);
			}
			return 0;
		}
		default:
			return BLE_ATT_ERR_UNLIKELY;
	}
}

/* ---------- rebuild orchestration ---------- */

/* Disconnect the current peer (if any) and wait briefly for the disconnect
   event to fire. Required before ble_gatts_reset(). */
static void disconnect_and_wait(void) {
	if (!is_connected)
		return;
	if (disc_sem)
		xSemaphoreTake(disc_sem, 0); // drain stale signals
	ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
	if (disc_sem) {
		xSemaphoreTake(disc_sem, pdMS_TO_TICKS(500));
	} else {
		// Fallback: small delay.
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/* Tear down GATT, rebuild from custom_services / custom_attr, restart adv. */
static custom_ble_result_t gatt_rebuild(void) {
	if (!has_started || !host_synced)
		return CUSTOM_BLE_NOT_STARTED;

	disconnect_and_wait();
	ble_gap_adv_stop();

	int rc = ble_gatts_reset();
	if (rc != 0)
		return CUSTOM_BLE_ESP_ERROR;

	ble_svc_gap_init();
	ble_svc_gatt_init();
	ble_store_config_init();
	ble_svc_gap_device_name_set(device_name);

	// Mark all service/attr slots as un-initialized; the register cb will
	// re-populate handles in declaration order.
	for (size_t i = 0; i < custom_service_len; i++) {
		custom_services[i].initialized = false;
	}
	for (size_t i = 0; i < custom_attr_len; i++) {
		custom_attr[i].initialized = false;
	}
	reg_cur_service      = 0;
	reg_cur_chr_attr_idx = 0;
	reg_cur_dsc_attr_idx = 0;

	if (build_service_tables() != 0) {
		return CUSTOM_BLE_INTERNAL_ERROR;
	}

	if (custom_service_len > 0) {
		rc = ble_gatts_count_cfg(built_svcs);
		if (rc != 0)
			return CUSTOM_BLE_ESP_ERROR;
		rc = ble_gatts_add_svcs(built_svcs);
		if (rc != 0)
			return CUSTOM_BLE_ESP_ERROR;
	}

	rc = ble_gatts_start();
	if (rc != 0)
		return CUSTOM_BLE_ESP_ERROR;

	start_advertising_with_cb();
	return CUSTOM_BLE_OK;
}

/* ---------- public API ---------- */

custom_ble_result_t custom_ble_set_name(const char *name) {
	if (init_result != CUSTOM_BLE_OK)
		return CUSTOM_BLE_INIT_FAILED;
	if (has_started)
		return CUSTOM_BLE_ALREADY_STARTED;
	size_t len = strlen(name);
	if (len > CUSTOM_BLE_MAX_NAME_LEN)
		return CUSTOM_BLE_NAME_TOO_LONG;
	strcpy(device_name, name);
	return CUSTOM_BLE_OK;
}

custom_ble_result_t custom_ble_update_adv(
	bool use_raw, size_t adv_len, const uint8_t adv_data_raw[adv_len],
	size_t scan_rsp_len, const uint8_t scan_rsp_data_raw[scan_rsp_len]
) {
	use_custom_adv_data = use_raw;
	if (use_custom_adv_data) {
		if ((adv_data_raw != NULL && adv_len > 31)
			|| (scan_rsp_data_raw != NULL && scan_rsp_len > 31)) {
			return CUSTOM_BLE_TOO_LONG;
		}
		if (adv_data_raw != NULL) {
			memcpy(ble_adv_data_raw, adv_data_raw, adv_len);
			ble_adv_data_raw_len = adv_len;
		}
		if (scan_rsp_data_raw != NULL) {
			memcpy(ble_scan_rsp_data_raw, scan_rsp_data_raw, scan_rsp_len);
			ble_scan_rsp_data_raw_len = scan_rsp_len;
		}
	}
	if (has_started) {
		ble_gap_adv_stop();
		start_advertising_with_cb();
	}
	return CUSTOM_BLE_OK;
}

void custom_ble_set_attr_write_handler(attr_write_cb_t cb) {
	attr_write_cb = cb;
}

custom_ble_result_t custom_ble_add_service(
	esp_bt_uuid_t service_uuid, uint16_t chr_count,
	const ble_chr_definition_t chr[chr_count], service_handles_cb_t handles_cb
) {
	if (init_result != CUSTOM_BLE_OK)
		return CUSTOM_BLE_INIT_FAILED;
	if (!has_started)
		return CUSTOM_BLE_NOT_STARTED;
	if (custom_service_len + 1 > service_capacity) {
		return CUSTOM_BLE_TOO_MANY_SERVICES;
	}

	uint16_t total_attrs = chr_count;
	for (size_t i = 0; i < chr_count; i++)
		total_attrs += chr[i].descr_count;
	if (custom_attr_len + total_attrs > chr_descr_capacity) {
		return CUSTOM_BLE_TOO_MANY_CHR_AND_DESCR;
	}

	// Stage the new service + attrs into the internal tables.
	custom_ble_id_t svc_idx  = (custom_ble_id_t)custom_service_len;
	custom_services[svc_idx] = (service_instance_t){
		.uuid        = service_uuid,
		.initialized = false,
	};
	custom_service_len++;

	size_t first_new_attr = custom_attr_len;

	for (size_t i = 0; i < chr_count; i++) {
		// Allocate value buffer for the chr. value_max_len of 0 is allowed
		// (e.g. notify-only zero-payload chr); we still allocate 1 byte
		// to avoid NULL ptr in access cb checks.
		size_t buf_sz = chr[i].value_max_len;
		if (buf_sz == 0)
			buf_sz = 1;
		uint8_t *buf = calloc(1, buf_sz);
		if (!buf)
			goto rollback;
		if (chr[i].value && chr[i].value_len > 0) {
			uint16_t cp = chr[i].value_len;
			if (cp > chr[i].value_max_len)
				cp = chr[i].value_max_len;
			memcpy(buf, chr[i].value, cp);
		}

		custom_attr[custom_attr_len++] = (attr_instance_t){
			.service_index = svc_idx,
			.uuid          = chr[i].uuid,
			.type          = CUSTOM_BLE_TYPE_CHR,
			.perm          = chr[i].perm,
			.prop          = chr[i].property,
			.value_max_len = chr[i].value_max_len,
			.value_len     = chr[i].value_len,
			.value         = buf,
			.initialized   = false,
		};

		for (size_t j = 0; j < chr[i].descr_count; j++) {
			const ble_desc_definition_t *d = &chr[i].descriptors[j];
			size_t dbuf_sz                 = d->value_max_len;
			if (dbuf_sz == 0)
				dbuf_sz = 1;
			uint8_t *dbuf = calloc(1, dbuf_sz);
			if (!dbuf)
				goto rollback;
			if (d->value && d->value_len > 0) {
				uint16_t cp = d->value_len;
				if (cp > d->value_max_len)
					cp = d->value_max_len;
				memcpy(dbuf, d->value, cp);
			}
			custom_attr[custom_attr_len++] = (attr_instance_t){
				.service_index = svc_idx,
				.uuid          = d->uuid,
				.type          = CUSTOM_BLE_TYPE_DESCR,
				.perm          = d->perm,
				.value_max_len = d->value_max_len,
				.value_len     = d->value_len,
				.value         = dbuf,
				.initialized   = false,
			};
		}
	}

	custom_ble_result_t rb = gatt_rebuild();
	if (rb != CUSTOM_BLE_OK)
		goto rollback;

	// Build the handles list for the callback: service handle, then each
	// chr's val_handle, with that chr's dsc handles immediately following.
	uint16_t handles_count = 1 + total_attrs;
	uint16_t *handles      = malloc(handles_count * sizeof(uint16_t));
	if (!handles) {
		// The service is registered fine on the BLE side; we just can't tell
		// the caller. Return INTERNAL_ERROR.
		return CUSTOM_BLE_INTERNAL_ERROR;
	}
	uint16_t hi   = 0;
	handles[hi++] = custom_services[svc_idx].service_handle;
	for (size_t a = first_new_attr; a < custom_attr_len; a++) {
		handles[hi++] = custom_attr[a].handle;
	}
	if (handles_cb)
		handles_cb(handles_count, handles);
	free(handles);
	return CUSTOM_BLE_OK;

rollback:
	// Drop any partial attrs we staged for this service, then drop the
	// service entry itself.
	for (size_t a = first_new_attr; a < custom_attr_len; a++) {
		free(custom_attr[a].value);
		custom_attr[a].value = NULL;
	}
	custom_attr_len = first_new_attr;
	custom_service_len--;
	return CUSTOM_BLE_ESP_ERROR;
}

custom_ble_result_t custom_ble_remove_service(uint16_t service_handle) {
	if (!has_started)
		return CUSTOM_BLE_NOT_STARTED;

	custom_ble_id_t svc_idx;
	if (!get_service_index(service_handle, &svc_idx)) {
		return CUSTOM_BLE_INVALID_HANDLE;
	}
	if (svc_idx != custom_service_len - 1)
		return CUSTOM_BLE_SERVICE_NOT_LAST;

	// Drop attrs belonging to the last service and free their value buffers.
	size_t new_attr_len = custom_attr_len;
	for (size_t i = 0; i < custom_attr_len; i++) {
		if (custom_attr[i].service_index == svc_idx) {
			if (i < new_attr_len)
				new_attr_len = i;
			free(custom_attr[i].value);
			custom_attr[i].value = NULL;
		}
	}
	custom_attr_len = new_attr_len;
	custom_service_len--;

	return gatt_rebuild();
}

custom_ble_result_t custom_ble_get_attr_value(
	uint16_t attr_handle, uint16_t *length, const uint8_t **value
) {
	if (!has_started)
		return CUSTOM_BLE_NOT_STARTED;
	int16_t i = get_attr_index_by_handle(attr_handle);
	if (i < 0)
		return CUSTOM_BLE_INVALID_HANDLE;
	if (length)
		*length = custom_attr[i].value_len;
	if (value)
		*value = custom_attr[i].value;
	return CUSTOM_BLE_OK;
}

custom_ble_result_t custom_ble_set_attr_value(
	uint16_t attr_handle, uint16_t length, const uint8_t value[length]
) {
	if (!has_started)
		return CUSTOM_BLE_NOT_STARTED;
	int16_t i = get_attr_index_by_handle(attr_handle);
	if (i < 0)
		return CUSTOM_BLE_INVALID_HANDLE;

	attr_instance_t *a = &custom_attr[i];
	if (length > a->value_max_len)
		length = a->value_max_len;
	memcpy(a->value, value, length);
	a->value_len = length;

	if (is_connected && a->type == CUSTOM_BLE_TYPE_CHR) {
		if (a->prop & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
			struct os_mbuf *om = ble_hs_mbuf_from_flat(a->value, a->value_len);
			if (om) {
				int rc = ble_gattc_notify_custom(conn_handle, a->handle, om);
				if (rc != 0)
					return CUSTOM_BLE_ESP_ERROR;
			}
		}
		if (a->prop & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
			struct os_mbuf *om = ble_hs_mbuf_from_flat(a->value, a->value_len);
			if (om) {
				int rc = ble_gattc_indicate_custom(conn_handle, a->handle, om);
				if (rc != 0)
					return CUSTOM_BLE_ESP_ERROR;
			}
		}
	}
	return CUSTOM_BLE_OK;
}

uint16_t custom_ble_service_count(void) {
	return (uint16_t)custom_service_len;
}

uint16_t custom_ble_get_services(
	uint16_t capacity, uint16_t service_handles[capacity]
) {
	if (!has_started)
		return 0;
	uint16_t n = (custom_service_len < capacity) ? custom_service_len
												 : capacity;
	for (uint16_t i = 0; i < n; i++) {
		service_handles[i] = custom_services[i].service_handle;
	}
	return n;
}

int16_t custom_ble_attr_count(uint16_t service_handle) {
	if (!has_started)
		return -1;
	custom_ble_id_t idx;
	if (!get_service_index(service_handle, &idx))
		return -1;
	int16_t count = 0;
	for (size_t i = 0; i < custom_attr_len; i++) {
		if (custom_attr[i].service_index == idx)
			count++;
	}
	return count;
}

custom_ble_result_t custom_ble_get_attrs(
	uint16_t service_handle, uint16_t capacity,
	uint16_t service_handles[capacity], uint16_t *written_count
) {
	if (!has_started)
		return CUSTOM_BLE_NOT_STARTED;
	custom_ble_id_t idx;
	if (!get_service_index(service_handle, &idx))
		return CUSTOM_BLE_INVALID_HANDLE;

	uint16_t w = 0;
	for (size_t i = 0; i < custom_attr_len && w < capacity; i++) {
		if (custom_attr[i].service_index == idx) {
			service_handles[w++] = custom_attr[i].handle;
		}
	}
	if (written_count)
		*written_count = w;
	return CUSTOM_BLE_OK;
}

bool custom_ble_started(void) {
	return has_started;
}

/* ---------- start: bring up controller + host ---------- */

static void on_sync(void) {
	if (ble_hs_util_ensure_addr(0) != 0)
		return;
	if (ble_hs_id_infer_auto(0, &own_addr_type) != 0)
		return;
	apply_tx_power();
	host_synced = true;
	ble_svc_gap_device_name_set(device_name);
	start_advertising_with_cb();
}

static void on_reset(int reason) {
	(void)reason;
	host_synced = false;
}

static void host_task(void *param) {
	(void)param;
	nimble_port_run();
	nimble_port_freertos_deinit();
}

custom_ble_result_t custom_ble_start(void) {
	if (init_result != CUSTOM_BLE_OK)
		return CUSTOM_BLE_INIT_FAILED;
	if (has_started)
		return CUSTOM_BLE_ALREADY_STARTED;

	disc_sem = xSemaphoreCreateBinary();
	if (!disc_sem)
		return CUSTOM_BLE_ERROR;

	esp_err_t err = nimble_port_init();
	if (err != ESP_OK)
		return CUSTOM_BLE_ESP_ERROR;

	ble_hs_cfg.reset_cb          = on_reset;
	ble_hs_cfg.sync_cb           = on_sync;
	ble_hs_cfg.gatts_register_cb = on_gatts_register;
	ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;
	ble_hs_cfg.sm_io_cap         = BLE_SM_IO_CAP_NO_IO;
	ble_hs_cfg.sm_bonding        = 0;
	ble_hs_cfg.sm_mitm           = 0;
	ble_hs_cfg.sm_sc             = 0;

	ble_svc_gap_init();
	ble_svc_gatt_init();
	ble_svc_gap_device_name_set(device_name);

	has_started = true;
	nimble_port_freertos_init(host_task);
	return CUSTOM_BLE_OK;
}

void custom_ble_init(void) {
	service_capacity   = backup.config.ble_service_capacity;
	chr_descr_capacity = backup.config.ble_chr_descr_capacity;

	if (service_capacity > 0) {
		custom_services = calloc(service_capacity, sizeof(service_instance_t));
		if (!custom_services) {
			init_result = CUSTOM_BLE_ERROR;
			return;
		}
	}
	if (chr_descr_capacity > 0) {
		custom_attr = calloc(chr_descr_capacity, sizeof(attr_instance_t));
		if (!custom_attr) {
			init_result = CUSTOM_BLE_ERROR;
			return;
		}
	}

	memcpy(device_name, (char *)backup.config.ble_name, 9);
	device_name[9] = '\0';

	init_result = CUSTOM_BLE_OK;
}
