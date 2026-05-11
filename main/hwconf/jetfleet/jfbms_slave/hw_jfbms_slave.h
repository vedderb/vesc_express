/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_JETFLEET_JFBMS_SLAVE_H_
#define MAIN_HWCONF_JETFLEET_JFBMS_SLAVE_H_

#include "adc.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "datatypes.h"

#define HW_NAME						"JFBMS_SLAVE"
#define HW_TARGET					"esp32c3"

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_IS_SLAVE
#define HW_INIT_HOOK()				hw_init()
#define HW_CAN_PING_SCAN_ENABLED	0  // Disable VESC CAN ping scan - slave uses 11-bit protocol
#define HW_CAN_NO_ACK_MODE			0  // Normal ACK mode for reliable communication
#define HW_CAN_FILTER_CONFIG(cfg)	hw_can_get_filter_config(cfg)
#define USER_EXTENSION_STORAGE_SIZE	50 // Extra slots for hw extensions (default 350 not enough)
//#define HW_POST_LISPIF_HOOK()		vTaskDelay(200);

// Configuration overrides
#define OVR_CONF_PARSER_C			"jfbms_slave_confparser.c"
#define OVR_CONF_PARSER_H			"jfbms_slave_confparser.h"
#define OVR_CONF_XML_C				"jfbms_slave_confxml.c"
#define OVR_CONF_XML_H				"jfbms_slave_confxml.h"
#define OVR_CONF_DEFAULT			"jfbms_slave_conf_default.h"
#define OVR_CONF_SERIALIZE			jfbms_slave_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		jfbms_slave_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		jfbms_slave_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763459

typedef enum {
	NTC_RES_4_7K = 0,
	NTC_RES_5K,
	NTC_RES_10K,
	NTC_RES_20K,
	NTC_RES_22K,
	NTC_RES_47K,
	NTC_RES_50K,
	NTC_RES_100K,
	NTC_RES_200K
} NTC_RES;

typedef struct {
	int slave_id;           // Slave ID on BMS CAN bus (1-8)
	CAN_BAUD can_baud_rate; // CAN baud rate (should be 500K per protocol)
	int cells_ic1;          // Cells on BQ76952 IC1 (3-16)
	int cells_ic2;          // Cells on BQ76952 IC2 (0-16, 0=single chip)
	NTC_RES temp_res;       // NTC resistance value at 25C
	uint16_t temp_beta;     // NTC B-constant (B25-85 recommended)

	// Compatibility fields (not used by slave, but needed for compilation)
	int controller_id;      // Not used - slave uses slave_id instead
	int can_status_rate_hz; // Not used by slave
	WIFI_MODE wifi_mode;    // Not used by slave
	char wifi_sta_ssid[36]; // Not used by slave
	char wifi_sta_key[26];  // Not used by slave
	char wifi_ap_ssid[36];  // Not used by slave
	char wifi_ap_key[26];   // Not used by slave
	bool use_tcp_local;     // Not used by slave
	bool use_tcp_hub;       // Not used by slave
	char tcp_hub_url[36];   // Not used by slave
	uint16_t tcp_hub_port;  // Not used by slave
	char tcp_hub_id[26];    // Not used by slave
	char tcp_hub_pass[26];  // Not used by slave
	BLE_MODE ble_mode;      // Not used by slave
	char ble_name[9];       // Not used by slave
	uint32_t ble_pin;       // Not used by slave
	uint32_t ble_service_capacity;    // Not used by slave
	uint32_t ble_chr_descr_capacity;  // Not used by slave
} main_config_t;

// Default setting Overrides
#define HW_DEFAULT_ID				3

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// I2C pins
#define PIN_SDA						21
#define PIN_SCL						20

// BQ communication enable pins (active LOW)
// During init: only BQ1 enabled while changing its I2C address from 0x08 to 0x10
// After init: both enabled (BQ1 at 0x10, BQ2 at 0x08 - different addresses)
#define PIN_BQ1_EN					0	// Pull LOW to enable BQ1 communication
#define PIN_BQ2_EN					1	// Pull LOW to enable BQ2 communication

// Buzzer
#define PIN_BUZZER					3

// Parameters
#define HW_R_SHUNT					0.0002

// Functions
void hw_init(void);
bool hw_can_get_filter_config(twai_filter_config_t *cfg);

#endif /* MAIN_HWCONF_JETFLEET_JFBMS_SLAVE_H_ */
