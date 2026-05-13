/*
	BLE host stack compatibility shim.

	On Bluedroid builds this includes the real esp_bt / esp_gap / esp_gatt
	headers.

	On NimBLE builds the Bluedroid headers are not available, so this defines
	the public type and macro subset used by custom_ble.h and
	lispif_ble_extensions.c. The concrete translation to NimBLE types happens in
	custom_ble_nimble.c / comm_ble_nimble.c.
*/

#ifndef MAIN_BLE_BLE_COMPAT_H_
#define MAIN_BLE_BLE_COMPAT_H_

#include "sdkconfig.h"

#if defined(CONFIG_BT_BLUEDROID_ENABLED)

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"

#elif defined(CONFIG_BT_NIMBLE_ENABLED)

#include <stdint.h>

#define ESP_UUID_LEN_16                  2
#define ESP_UUID_LEN_32                  4
#define ESP_UUID_LEN_128                 16

typedef struct {
	uint16_t len;
	union {
		uint16_t uuid16;
		uint32_t uuid32;
		uint8_t uuid128[ESP_UUID_LEN_128];
	} uuid;
} __attribute__((packed)) esp_bt_uuid_t;

typedef uint16_t esp_gatt_perm_t;
#define ESP_GATT_PERM_READ               (1 << 0)
#define ESP_GATT_PERM_READ_ENCRYPTED     (1 << 1)
#define ESP_GATT_PERM_READ_ENC_MITM      (1 << 2)
#define ESP_GATT_PERM_WRITE              (1 << 4)
#define ESP_GATT_PERM_WRITE_ENCRYPTED    (1 << 5)
#define ESP_GATT_PERM_WRITE_ENC_MITM     (1 << 6)
#define ESP_GATT_PERM_WRITE_SIGNED       (1 << 7)
#define ESP_GATT_PERM_WRITE_SIGNED_MITM  (1 << 8)

typedef uint8_t esp_gatt_char_prop_t;
#define ESP_GATT_CHAR_PROP_BIT_BROADCAST (1 << 0)
#define ESP_GATT_CHAR_PROP_BIT_READ      (1 << 1)
#define ESP_GATT_CHAR_PROP_BIT_WRITE_NR  (1 << 2)
#define ESP_GATT_CHAR_PROP_BIT_WRITE     (1 << 3)
#define ESP_GATT_CHAR_PROP_BIT_NOTIFY    (1 << 4)
#define ESP_GATT_CHAR_PROP_BIT_INDICATE  (1 << 5)
#define ESP_GATT_CHAR_PROP_BIT_AUTH      (1 << 6)
#define ESP_GATT_CHAR_PROP_BIT_EXT_PROP  (1 << 7)

#define ADV_TYPE_IND 0x00

#define BLE_ADDR_TYPE_PUBLIC 0x00
#define BLE_ADDR_TYPE_RANDOM 0x01

#define ADV_CHNL_37  0x01
#define ADV_CHNL_38  0x02
#define ADV_CHNL_39  0x04
#define ADV_CHNL_ALL 0x07

#define ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY 0x00

typedef struct {
	uint16_t adv_int_min;
	uint16_t adv_int_max;
	uint8_t adv_type;
	uint8_t own_addr_type;
	uint8_t peer_addr[6];
	uint8_t peer_addr_type;
	uint8_t channel_map;
	uint8_t adv_filter_policy;
} esp_ble_adv_params_t;

#else
#error "Neither CONFIG_BT_BLUEDROID_ENABLED nor CONFIG_BT_NIMBLE_ENABLED is set"
#endif

#endif /* MAIN_BLE_BLE_COMPAT_H_ */
