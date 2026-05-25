/*
	NimBLE implementation of the VESC Tool BLE channel.

	The GATT layout matches the Bluedroid version so VESC Tool sees the same
	Nordic UART-style service and RX / TX characteristics.
*/

#include "comm_ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "packet.h"
#include "commands.h"
#include "conf_general.h"
#include "main.h"

#define DEFAULT_BLE_MTU         20 // 23 - 3 ATT header
#define MAX_BLE_PAYLOAD         255
#define BLE_NOTIFY_MAX_RETRIES  8
#define BLE_NOTIFY_RETRY_MS     5
#define BLE_NOTIFY_CHUNK_GAP_MS 3

#ifndef HW_BLE_PWR_LVL
#if CONFIG_IDF_TARGET_ESP32C6
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P9
#else
#define HW_BLE_PWR_LVL ESP_PWR_LVL_P18
#endif
#endif
#ifndef HW_BLE_PWR_LVL_DEFAULT
#define HW_BLE_PWR_LVL_DEFAULT HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_ADV
#define HW_BLE_PWR_LVL_ADV HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_SCAN
#define HW_BLE_PWR_LVL_SCAN HW_BLE_PWR_LVL
#endif
#ifndef HW_BLE_PWR_LVL_CONN
#define HW_BLE_PWR_LVL_CONN HW_BLE_PWR_LVL
#endif

void ble_store_config_init(void);

static bool is_connected            = false;
static uint16_t ble_current_mtu     = DEFAULT_BLE_MTU;
static uint16_t conn_handle         = 0;
static uint16_t tx_char_val_handle  = 0;
static uint8_t own_addr_type        = 0;
static PACKET_STATE_t *packet_state = NULL;

/*
 * Service / characteristic UUIDs. Layout (LE byte order) is identical to
 * the Bluedroid build in comm_ble.c.
 *
 *   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *   RX char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E   (write / write-no-rsp)
 *   TX char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E   (read / notify)
 */
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x01, 0x00, 0x40, 0x6E
);

static const ble_uuid128_t rx_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x02, 0x00, 0x40, 0x6E
);

static const ble_uuid128_t tx_uuid = BLE_UUID128_INIT(
	0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5,
	0x03, 0x00, 0x40, 0x6E
);

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
);
static int gap_event_cb(struct ble_gap_event *event, void *arg);

static const struct ble_gatt_chr_def gatt_chrs[] = {
	{
		.uuid      = &rx_uuid.u,
		.access_cb = gatt_access_cb,
		.flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
	},
	{
		.uuid       = &tx_uuid.u,
		.access_cb  = gatt_access_cb,
		.flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
		.val_handle = &tx_char_val_handle,
	},
	{0},
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
	{
		.type            = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid            = &svc_uuid.u,
		.characteristics = gatt_chrs,
	},
	{0},
};

static void apply_tx_power(void) {
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, HW_BLE_PWR_LVL_ADV);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, HW_BLE_PWR_LVL_SCAN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, HW_BLE_PWR_LVL_DEFAULT);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, HW_BLE_PWR_LVL_CONN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, HW_BLE_PWR_LVL_CONN);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, HW_BLE_PWR_LVL_CONN);
}

static uint8_t ble_name_len(void) {
	uint8_t len = 0;
	while (len < (sizeof(backup.config.ble_name) - 1)
		   && backup.config.ble_name[len] != '\0') {
		len++;
	}
	return len;
}

static void start_advertising(void) {
	struct ble_gap_adv_params adv_params = {0};
	struct ble_hs_adv_fields adv_fields  = {0};
	struct ble_hs_adv_fields rsp_fields  = {0};

	const char *name = (const char *)backup.config.ble_name;
	uint8_t name_len = ble_name_len();

	// Match the Bluedroid advertisement layout. The stored BLE name is capped
	// to 8 display bytes, so flags + complete VESC UART UUID + name fits in
	// the 31-byte legacy advertisement.
	adv_fields.flags        = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
	adv_fields.uuids128     = (ble_uuid128_t *)&svc_uuid;
	adv_fields.num_uuids128 = 1;
	adv_fields.uuids128_is_complete = 1;
	adv_fields.name                  = (uint8_t *)name;
	adv_fields.name_len              = name_len;
	adv_fields.name_is_complete      = 1;

	int rc = ble_gap_adv_set_fields(&adv_fields);
	if (rc != 0) {
		return;
	}

	rsp_fields.tx_pwr_lvl_is_present = 1;
	rsp_fields.tx_pwr_lvl            = BLE_HS_ADV_TX_PWR_LVL_AUTO;
	rsp_fields.name                  = (uint8_t *)name;
	rsp_fields.name_len              = name_len;
	rsp_fields.name_is_complete      = 1;
	ble_gap_adv_rsp_set_fields(&rsp_fields);

	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	adv_params.itvl_min  = 0x20;
	adv_params.itvl_max  = 0x40;

	ble_gap_adv_start(
		own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL
	);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
	switch (event->type) {
		case BLE_GAP_EVENT_CONNECT:
			if (event->connect.status == 0) {
				conn_handle     = event->connect.conn_handle;
				is_connected    = true;
				ble_current_mtu = DEFAULT_BLE_MTU;
				LED_BLUE_ON();
				apply_tx_power();

				if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
					ble_gap_security_initiate(conn_handle);
				}
			} else {
				start_advertising();
			}
			return 0;

		case BLE_GAP_EVENT_DISCONNECT:
			is_connected = false;
			LED_BLUE_OFF();
			start_advertising();
			return 0;

		case BLE_GAP_EVENT_MTU:
			if (event->mtu.value <= 3) {
				ble_current_mtu = DEFAULT_BLE_MTU;
			} else {
				uint16_t payload_mtu = event->mtu.value - 3; // strip ATT header
				ble_current_mtu      = payload_mtu > MAX_BLE_PAYLOAD
						 ? MAX_BLE_PAYLOAD
						 : payload_mtu;
			}
			return 0;

		case BLE_GAP_EVENT_PASSKEY_ACTION: {
			if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
				struct ble_sm_io io = {
					.action  = BLE_SM_IOACT_DISP,
					.passkey = backup.config.ble_pin,
				};
				ble_sm_inject_io(event->passkey.conn_handle, &io);
			}
			return 0;
		}

		case BLE_GAP_EVENT_REPEAT_PAIRING: {
			struct ble_gap_conn_desc desc;
			if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc)
				== 0) {
				ble_store_util_delete_peer(&desc.peer_id_addr);
			}
			return BLE_GAP_REPEAT_PAIRING_RETRY;
		}

		case BLE_GAP_EVENT_SUBSCRIBE:
		case BLE_GAP_EVENT_NOTIFY_TX:
		case BLE_GAP_EVENT_CONN_UPDATE:
		default:
			return 0;
	}
}

static int gatt_access_cb(
	uint16_t conn_h, uint16_t attr_h, struct ble_gatt_access_ctxt *ctxt,
	void *arg
) {
	(void)arg;

	switch (ctxt->op) {
		case BLE_GATT_ACCESS_OP_WRITE_CHR: {
			// Feed packet bytes regardless of which char (RX char in practice;
			// the TX char declares WRITE so VESC Tool can negotiate, but writes
			// to TX are ignored at the protocol layer).
			uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
			uint8_t buf[MAX_BLE_PAYLOAD];
			if (len > sizeof(buf))
				len = sizeof(buf);

			uint16_t out_len = 0;
			ble_hs_mbuf_to_flat(ctxt->om, buf, len, &out_len);

			if (packet_state && ble_uuid_cmp(ctxt->chr->uuid, &rx_uuid.u) == 0) {
				for (uint16_t i = 0; i < out_len; i++) {
					packet_process_byte(buf[i], packet_state);
				}
			}
			conn_handle = conn_h;
			return 0;
		}
		case BLE_GATT_ACCESS_OP_READ_CHR:
			// Reading the TX char returns no current value; VESC Tool only
			// reads it to discover the value handle; the data flows via notify.
			return 0;

		default:
			return BLE_ATT_ERR_UNLIKELY;
	}
}

static void on_sync(void) {
	int rc = ble_hs_util_ensure_addr(0);
	if (rc != 0) {
		return;
	}

	rc = ble_hs_id_infer_auto(0, &own_addr_type);
	if (rc != 0) {
		return;
	}

	apply_tx_power();
	start_advertising();
}

static void on_reset(int reason) {
	(void)reason;
}

static void host_task(void *param) {
	(void)param;
	nimble_port_run();
	nimble_port_freertos_deinit();
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_ble_send_packet);
}

static void free_packet_state(void) {
	if (packet_state) {
		free(packet_state);
		packet_state = NULL;
	}
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	if (!is_connected || tx_char_val_handle == 0) {
		return;
	}

	uint16_t bytes_sent = 0;
	while (bytes_sent < len) {
		uint16_t chunk = len - bytes_sent;
		if (chunk > ble_current_mtu) {
			chunk = ble_current_mtu;
		}

		bool sent = false;

		for (int attempt = 0; attempt <= BLE_NOTIFY_MAX_RETRIES; attempt++) {
			struct os_mbuf *om =
				ble_hs_mbuf_from_flat(buffer + bytes_sent, chunk);
			if (om == NULL) {
				vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_RETRY_MS));
				continue;
			}

			int rc =
				ble_gattc_notify_custom(conn_handle, tx_char_val_handle, om);
			if (rc == 0) {
				sent = true;
				break;
			}

			// notify_custom consumes the mbuf. Give the host/controller queues
			// time to drain before retrying long VESC packets such as config XML.
			vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_RETRY_MS));
		}

		if (!sent) {
			return;
		}

		bytes_sent += chunk;

		if (bytes_sent < len) {
			vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_CHUNK_GAP_MS));
		}
	}
}

void comm_ble_init(void) {
	packet_state = calloc(1, sizeof(PACKET_STATE_t));
	if (!packet_state) {
		return;
	}

	packet_init(send_packet_raw, process_packet, packet_state);

	esp_err_t err = nimble_port_init();
	if (err != ESP_OK) {
		free_packet_state();
		return;
	}

	ble_hs_cfg.reset_cb          = on_reset;
	ble_hs_cfg.sync_cb           = on_sync;
	ble_hs_cfg.gatts_register_cb = NULL;
	ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		ble_hs_cfg.sm_io_cap       = BLE_SM_IO_CAP_DISP_ONLY;
		ble_hs_cfg.sm_bonding      = 1;
		ble_hs_cfg.sm_mitm         = 1;
		ble_hs_cfg.sm_sc           = 1;
		ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC
			| BLE_SM_PAIR_KEY_DIST_ID;
		ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC
			| BLE_SM_PAIR_KEY_DIST_ID;
	} else {
		ble_hs_cfg.sm_io_cap  = BLE_SM_IO_CAP_NO_IO;
		ble_hs_cfg.sm_bonding = 0;
		ble_hs_cfg.sm_mitm    = 0;
		ble_hs_cfg.sm_sc      = 0;
	}

	ble_svc_gap_init();
	ble_svc_gatt_init();
	ble_store_config_init();

	int rc = ble_gatts_count_cfg(gatt_svcs);
	if (rc != 0) {
		nimble_port_deinit();
		free_packet_state();
		return;
	}
	rc = ble_gatts_add_svcs(gatt_svcs);
	if (rc != 0) {
		nimble_port_deinit();
		free_packet_state();
		return;
	}

	ble_svc_gap_device_name_set((const char *)backup.config.ble_name);

	nimble_port_freertos_init(host_task);
}

bool comm_ble_is_connected() {
	return is_connected;
}

int comm_ble_mtu_now(void) {
	return ble_current_mtu;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	if (!packet_state) {
		return;
	}

	packet_send_packet(data, len, packet_state);
}
