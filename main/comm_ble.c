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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"

#include "packet.h"
#include "commands.h"
#include "conf_general.h"
#include "main.h"

#define GATTS_CHAR_VAL_LEN_MAX		255
#define BLE_CHAR_COUNT				2
#define BLE_SERVICE_HANDLE_NUM		(1 + (3 * BLE_CHAR_COUNT))
#define ADV_CFG_FLAG				(1 << 0)
#define SCAN_RSP_CFG_FLAG			(1 << 1)

static bool is_connected = false;
static uint16_t ble_current_mtu = 20;

static uint16_t notify_conn_id = 0;
static esp_gatt_if_t notify_gatts_if;

static uint8_t adv_config_done = 0;

static uint8_t char1_str[GATTS_CHAR_VAL_LEN_MAX] = {0};
static uint8_t char2_str[GATTS_CHAR_VAL_LEN_MAX] = {0};

static esp_attr_value_t gatts_char1_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len     = sizeof(char1_str),
	.attr_value   = char1_str,
};

static esp_attr_value_t gatts_char2_val = {
	.attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
	.attr_len     = sizeof(char2_str),
	.attr_value   = char2_str,
};

typedef struct  {
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t gatts_if;
	esp_gatt_perm_t perm;

	uint16_t char_handle;
	esp_gatt_char_prop_t property;
	esp_bt_uuid_t char_uuid;

	uint16_t desc_handle;
	esp_bt_uuid_t desc_uuid;

	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
} gatts_profile_instance_t;

typedef struct  {
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t char_perm;
	esp_gatt_char_prop_t char_property;
	esp_attr_value_t *char_val;
	esp_attr_control_t *char_control;
	esp_gatts_cb_t char_read_callback;
	esp_gatts_cb_t char_write_callback;

	uint16_t desc_handle;
	esp_bt_uuid_t desc_uuid;
	esp_gatt_perm_t desc_perm;
	esp_attr_value_t *desc_val;
	esp_attr_control_t *desc_control;
	esp_gatts_cb_t desc_read_callback;
	esp_gatts_cb_t desc_write_callback;
} gatts_characteristic_instance_t;

static uint8_t ble_service_uuid128[ESP_UUID_LEN_128] = {
		0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static uint32_t ble_add_char_position;

static esp_ble_adv_data_t ble_adv_data = {
	.set_scan_rsp = false,
	.include_name = true,
	.include_txpower = false,
	.min_interval = 0x06,
	.max_interval = 0x30,
	.appearance = 0x00,
	.manufacturer_len = 0,
	.p_manufacturer_data =  NULL,
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = ESP_UUID_LEN_128,
	.p_service_uuid = ble_service_uuid128,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t ble_rsp_data = {
	.set_scan_rsp = true,
	.include_name = true,
	.include_txpower = true,
	.min_interval = 0x06,
	.max_interval = 0x30,
	.appearance = 0x00,
	.manufacturer_len = 0,
	.p_manufacturer_data =  NULL,
	.service_data_len = 0,
	.p_service_data = NULL,
	.service_uuid_len = ESP_UUID_LEN_128,
	.p_service_uuid = ble_service_uuid128,
	.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t ble_adv_params = {
	.adv_int_min = 0x20,
	.adv_int_max = 0x40,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static gatts_profile_instance_t gatts_profile = {
		.gatts_if = ESP_GATT_IF_NONE,
};

static PACKET_STATE_t *packet_state;

static void char1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void char1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void descr1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void descr1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void char2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void char2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void descr2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void descr2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static gatts_characteristic_instance_t ble_chars[BLE_CHAR_COUNT] = {
{
	.char_uuid.len = ESP_UUID_LEN_128,  // RX
	.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E },
	.char_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
	.char_property = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
	.char_val = &gatts_char1_val,
	.char_control = NULL,
	.char_handle = 0,
	.char_read_callback = char1_read_handler,
	.char_write_callback = char1_write_handler,

	.desc_uuid.len = ESP_UUID_LEN_16,
	.desc_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
	.desc_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
	.desc_val = NULL,
	.desc_control = NULL,
	.desc_handle = 0,
	.desc_read_callback = descr1_read_handler,
	.desc_write_callback = descr1_write_handler,
},
{
	.char_uuid.len = ESP_UUID_LEN_128, // TX
	.char_uuid.uuid.uuid128 =  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E },
	.char_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
	.char_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
	.char_val = &gatts_char2_val,
	.char_control = NULL,
	.char_handle = 0,
	.char_read_callback = char2_read_handler,
	.char_write_callback = char2_write_handler,

	.desc_uuid.len = ESP_UUID_LEN_16,
	.desc_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
	.desc_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE),
	.desc_val = NULL,
	.desc_control = NULL,
	.desc_handle = 0,
	.desc_read_callback = descr2_read_handler,
	.desc_write_callback = descr2_write_handler,
}
};

static esp_gatt_rsp_t create_rsp(esp_attr_value_t *attr, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_rsp_t rsp;
	memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	rsp.attr_value.handle = param->read.handle;
	if (attr != NULL) {
		rsp.attr_value.len = attr->attr_len;
		for (uint32_t i = 0;i < attr->attr_len && i < attr->attr_max_len;i++) {
			rsp.attr_value.value[i] = attr->attr_value[i];
		}
	}

	return rsp;
}

static void char1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_rsp_t rsp = create_rsp(ble_chars[0].char_val, param);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

static void char2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_rsp_t rsp = create_rsp(ble_chars[1].char_val, param);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

static void descr1_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_rsp_t rsp = create_rsp(ble_chars[0].desc_val, param);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

static void descr2_read_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_rsp_t rsp = create_rsp(ble_chars[1].desc_val, param);
	esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

static void write_attr(esp_attr_value_t *attr, esp_ble_gatts_cb_param_t *param) {
	if (attr != NULL) {
		attr->attr_len = param->write.len;
		for (uint32_t i = 0;i < param->write.len;i++) {
			attr->attr_value[i] = param->write.value[i];
		}
	}
}

static void char1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	write_attr(ble_chars[0].char_val, param);

	if (ble_chars[0].char_val != NULL) {
		for (int i = 0; i < param->write.len; ++i) {
			packet_process_byte(param->write.value[i], packet_state);
		}
	}

	notify_gatts_if = gatts_if;
	notify_conn_id = param->write.conn_id;

	if (param->write.need_rsp) {
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	}
}

static void char2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	write_attr(ble_chars[1].char_val, param);
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

static void descr1_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	write_attr(ble_chars[0].desc_val, param);
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

static void descr2_write_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	write_attr(ble_chars[1].desc_val, param);
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
}

static void gatts_check_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	uint16_t handle = 0;
	bool read = true;

	switch (event) {
	case ESP_GATTS_READ_EVT: {
		read = true;
		handle = param->read.handle;
		break;
	}

	case ESP_GATTS_WRITE_EVT: {
		read = false;
		handle = param->write.handle;
		break;
	}

	default:
		break;
	}

	for (uint32_t i=0; i < BLE_CHAR_COUNT;i++) {
		if (ble_chars[i].char_handle == handle) {
			if (read) {
				if (ble_chars[i].char_read_callback != NULL) {
					ble_chars[i].char_read_callback(event, gatts_if, param);
				}
			} else {
				if (ble_chars[i].char_write_callback != NULL) {
					ble_chars[i].char_write_callback(event, gatts_if, param);
				}
			}
			break;
		}

		if (ble_chars[i].desc_handle == handle) {
			if (read) {
				if (ble_chars[i].desc_read_callback != NULL) {
					ble_chars[i].desc_read_callback(event, gatts_if, param);
				}
			} else {
				if (ble_chars[i].desc_write_callback != NULL) {
					ble_chars[i].desc_write_callback(event, gatts_if, param);
				}
			}
			break;
		}
	}
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
	switch (event) {
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~ADV_CFG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&ble_adv_params);
		}
		break;

	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~SCAN_RSP_CFG_FLAG);
		if (adv_config_done == 0){
			esp_ble_gap_start_advertising(&ble_adv_params);
		}
		break;

	case ESP_GAP_BLE_SEC_REQ_EVT:
		esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
		break;

	default:
		break;
	}
}

static void gatts_add_char() {
	for (uint32_t i=0;i < BLE_CHAR_COUNT;i++) {
		if (ble_chars[i].char_handle == 0) {
			ble_add_char_position = i;

			esp_ble_gatts_add_char(
					gatts_profile.service_handle,
					&ble_chars[i].char_uuid,
					ble_chars[i].char_perm,
					ble_chars[i].char_property,
					ble_chars[i].char_val,
					ble_chars[i].char_control);
			break;
		}
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gatts_profile.gatts_if = gatts_if;
		} else {
			return;
		}
	}

	switch (event) {
	case ESP_GATTS_REG_EVT:
		gatts_profile.service_id.is_primary = true;
		gatts_profile.service_id.id.inst_id = 0x00;
		gatts_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;

		for (uint8_t i = 0;i < ESP_UUID_LEN_128;i++) {
			gatts_profile.service_id.id.uuid.uuid.uuid128[i] = ble_service_uuid128[i];
		}

		esp_ble_gap_set_device_name((char*)backup.config.ble_name);

		esp_ble_gap_config_adv_data(&ble_adv_data);
		adv_config_done |= ADV_CFG_FLAG;

		esp_ble_gap_config_adv_data(&ble_rsp_data);
		adv_config_done |= SCAN_RSP_CFG_FLAG;

		esp_ble_gatts_create_service(gatts_if, &gatts_profile.service_id, BLE_SERVICE_HANDLE_NUM);
		break;

	case ESP_GATTS_READ_EVT:
		gatts_check_callback(event, gatts_if, param);
		break;

	case ESP_GATTS_WRITE_EVT:
		gatts_check_callback(event, gatts_if, param);
		break;

	case ESP_GATTS_EXEC_WRITE_EVT:
	case ESP_GATTS_MTU_EVT:
		if (param->mtu.mtu == 0) {
			ble_current_mtu = 20;
		} else if (param->mtu.mtu > GATTS_CHAR_VAL_LEN_MAX) {
			ble_current_mtu = GATTS_CHAR_VAL_LEN_MAX;
		} else {
			ble_current_mtu = param->mtu.mtu;
		}
		break;

	case ESP_GATTS_CONF_EVT:
	case ESP_GATTS_UNREG_EVT:

		break;

	case ESP_GATTS_CREATE_EVT:
		gatts_profile.service_handle = param->create.service_handle;
		gatts_profile.char_uuid.len = ble_chars[0].char_uuid.len;
		gatts_profile.char_uuid.uuid.uuid16 = ble_chars[0].char_uuid.uuid.uuid16;

		esp_ble_gatts_start_service(gatts_profile.service_handle);
		gatts_add_char();
		break;

	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;

	case ESP_GATTS_ADD_CHAR_EVT:
		gatts_profile.char_handle = param->add_char.attr_handle;

		if (param->add_char.status == ESP_GATT_OK) {
			if (param->add_char.attr_handle != 0) {
				ble_chars[ble_add_char_position].char_handle = param->add_char.attr_handle;

				if (ble_chars[ble_add_char_position].desc_uuid.len !=0 &&
						ble_chars[ble_add_char_position].desc_handle == 0) {
					esp_ble_gatts_add_char_descr(
							gatts_profile.service_handle,
							&ble_chars[ble_add_char_position].desc_uuid,
							ble_chars[ble_add_char_position].desc_perm,
							ble_chars[ble_add_char_position].desc_val,
							ble_chars[ble_add_char_position].desc_control);
				} else {
					gatts_add_char();
				}
			}
		}
		break;

	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		if (param->add_char_descr.status == ESP_GATT_OK) {
			if (param->add_char.attr_handle != 0) {
				ble_chars[ble_add_char_position].desc_handle = param->add_char.attr_handle;
			}

			gatts_add_char();
		}
		break;

	case ESP_GATTS_DELETE_EVT:
		break;

	case ESP_GATTS_START_EVT:
		break;

	case ESP_GATTS_STOP_EVT:
		break;

	case ESP_GATTS_CONNECT_EVT:
		if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
			esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
		}

		gatts_profile.conn_id = param->connect.conn_id;
		is_connected = true;
		LED_BLUE_ON();

		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P18);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P18);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P18);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P18);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P18);
		esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P18);
		break;

	case ESP_GATTS_DISCONNECT_EVT:
		is_connected = false;
		LED_BLUE_OFF();
		esp_ble_gap_start_advertising(&ble_adv_params);
		break;

	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
	case ESP_GATTS_CONGEST_EVT:

	default:
		break;
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_ble_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	if (!is_connected) {
		return;
	}

	uint16_t bytes_sent = 0;

	while (bytes_sent < len) {
		uint8_t length = 0;
		if (len - bytes_sent > ble_current_mtu) {
			length = ble_current_mtu;
		} else {
			length = len - bytes_sent;
		}

		esp_ble_gatts_send_indicate(
				notify_gatts_if,
				notify_conn_id,
				ble_chars[1].char_handle,
				length,
				buffer + bytes_sent,
				false);

		bytes_sent += length;
	}
}

void comm_ble_init(void) {
	packet_state = calloc(1, sizeof(PACKET_STATE_t));
	packet_init(send_packet_raw, process_packet, packet_state);

	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		ble_chars[0].char_perm = (ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
		ble_chars[0].desc_perm = (ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
		ble_chars[1].char_perm = (ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
		ble_chars[1].desc_perm = (ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
	} else {
		ble_chars[0].char_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
		ble_chars[0].desc_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
		ble_chars[1].char_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
		ble_chars[1].desc_perm = (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
	}

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);

	esp_bt_controller_enable(ESP_BT_MODE_BLE);
	esp_bluedroid_init();
	esp_bluedroid_enable();

	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P18);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P18);

	esp_bt_dev_set_device_name((char*)backup.config.ble_name);

	esp_ble_gatts_register_callback(gatts_event_handler);
	esp_ble_gap_register_callback(gap_event_handler);
	esp_ble_gatts_app_register(0);

	if (backup.config.ble_mode == BLE_MODE_ENCRYPTED) {
		uint32_t passkey = backup.config.ble_pin;
		esp_ble_auth_req_t auth_req	= ESP_LE_AUTH_REQ_SC_MITM_BOND;
		esp_ble_io_cap_t iocap		= ESP_IO_CAP_OUT;
		uint8_t key_size			= 16;
		uint8_t init_key			= ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
		uint8_t rsp_key				= ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
		uint8_t auth_option			= ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
		uint8_t oob_support			= ESP_BLE_OOB_DISABLE;
		esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
		esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
	} else {
		esp_ble_auth_req_t auth_req	= ESP_LE_AUTH_NO_BOND;
		esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	}
}

bool comm_ble_is_connected() {
	return is_connected;
}

int comm_ble_mtu_now(void) {
	return ble_current_mtu;
}

void comm_ble_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, packet_state);
}
