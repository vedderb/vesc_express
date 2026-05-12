/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_JETFLEET_HW_JF_LINK_H_
#define MAIN_HWCONF_JETFLEET_HW_JF_LINK_H_

#include "datatypes.h"
#include "driver/gpio.h"

#define HW_NAME						"JF Link"
#define HW_TARGET					"esp32c3"

#define HW_INIT_HOOK()				hw_init()
#define HW_POST_LISPIF_HOOK()		hw_jf_link_start()

// Configuration overrides
#define OVR_CONF_PARSER_C			"jf_link_confparser.c"
#define OVR_CONF_PARSER_H			"jf_link_confparser.h"
#define OVR_CONF_XML_C				"jf_link_confxml.c"
#define OVR_CONF_XML_H				"jf_link_confxml.h"
#define OVR_CONF_DEFAULT			"jf_link_conf_default.h"
#define OVR_CONF_SERIALIZE			jf_link_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		jf_link_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		jf_link_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763459

typedef struct {
	int controller_id;
	CAN_BAUD can_baud_rate;
	int can_status_rate_hz;
	WIFI_MODE wifi_mode;
	char wifi_sta_ssid[36];
	char wifi_sta_key[26];
	char wifi_ap_ssid[36];
	char wifi_ap_key[26];
	bool use_tcp_local;
	bool use_tcp_hub;
	char tcp_hub_url[36];
	uint16_t tcp_hub_port;
	char tcp_hub_id[26];
	char tcp_hub_pass[26];
	BLE_MODE ble_mode;
	char ble_name[9];
	uint32_t ble_pin;
	uint32_t ble_service_capacity;
	uint32_t ble_chr_descr_capacity;

	int num_slaves;
	float slave_timeout_s;
	int max_bal_ch;
	float vc_balance_start;
	float vc_balance_end;
	float vc_balance_min;
} main_config_t;

// Default setting Overrides
#define HW_DEFAULT_ID				3

// LEDs: ESP32-C3 link board
#define LED_RED_PIN					2
#define LED_BLUE_PIN				3

#define LED_RED_ON()				gpio_set_level(LED_RED_PIN, 1)
#define LED_RED_OFF()				gpio_set_level(LED_RED_PIN, 0)

#define LED_BLUE_ON()				gpio_set_level(LED_BLUE_PIN, 1)
#define LED_BLUE_OFF()				gpio_set_level(LED_BLUE_PIN, 0)

// CAN: ESP32-C3 link board
#define CAN_TX_GPIO_NUM				1
#define CAN_RX_GPIO_NUM				0

// SD-card: ESP32-C3 link board
#define SD_PIN_MOSI					4
#define SD_PIN_MISO					6
#define SD_PIN_SCK					5
#define SD_PIN_CS					7

// UART: ESP32-C3 link board
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20

// Functions
void hw_init(void);
void hw_jf_link_start(void);

#endif /* MAIN_HWCONF_JETFLEET_HW_JF_LINK_H_ */
