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

#ifndef MAIN_HWCONF_VESC_VBMS16_H_
#define MAIN_HWCONF_VESC_VBMS16_H_

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME						"VBMS16"

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_INIT_HOOK()				hw_init()
//#define HW_POST_LISPIF_HOOK()		vTaskDelay(200);

// Configuration overrides
#define OVR_CONF_PARSER_C			"vbms16_confparser.c"
#define OVR_CONF_PARSER_H			"vbms16_confparser.h"
#define OVR_CONF_XML_C				"vbms16_confxml.c"
#define OVR_CONF_XML_H				"vbms16_confxml.h"
#define OVR_CONF_DEFAULT			"vbms16_conf_default.h"
#define OVR_CONF_SERIALIZE			vbms16_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		vbms16_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		vbms16_confparser_set_defaults_main_config_t
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

	// Cells on balance IC
	int cells_ic1;
	
	// Number of external temperature sensors
	int temp_num;

	// Battery amp hours
	float batt_ah;

	// Maximum simultaneous balancing channels
	int max_bal_ch;

	// Use amp hours for columb counting
	bool soc_use_ah;

	// Block sleep mode
	bool block_sleep;

	// Cell voltage when empty
	float vc_empty;

	// Cell voltage when full
	float vc_full;

	// Start balancing if cell voltage is this much above the minimum cell voltage
	float vc_balance_start;

	// Stop balancing when cell voltage is this much above the minimum cell voltage
	float vc_balance_end;

	// Start charging when max cell voltage is below this voltage
	float vc_charge_start;

	// End charging when max cell voltage is above this voltage
	float vc_charge_end;

	// Only allow charging if all cells are above this voltage
	float vc_charge_min;

	// Only allow balancing if all cells are above this voltage
	float vc_balance_min;

	// Only allow balancing when the current magnitude is below this value
	float balance_max_current;

	// Current must be above this magnitude for the Ah and Wh couters to run
	float min_current_ah_wh_cnt;

	// Enter sleep mode when the current magnitude is below this value
	float min_current_sleep;

	// Charge port voltage at which a charger is considered plugged in
	float v_charge_detect;

	// Only allow charging when the cell temperature is below this value
	float t_charge_max;
	
	// Only allow charging when the MOSFET temperature is below this value
	float t_charge_max_mos;

	// Regular sleep time
	float sleep_regular;

	// Long sleep time, for when SOC is low
	float sleep_long;

	// Stop charging when the charge current goes below this value
	float min_charge_current;

	// Maximum allowed charging current
	float max_charge_current;

	// Filter constant for SoC filter
	float soc_filter_const;

	// Do not allow balancing above this cell temperature
	float t_bal_max_cell;
	
	// Do not allow balancing above this balance IC temperature
	float t_bal_max_ic;

	// Only allow charging when the cell temperature is above this value
	float t_charge_min;

	// Enable temperature monitoring during charging
	bool t_charge_mon_en;
	
	// Maximum precharge time
	float psw_t_pchg;
	
	// Shortcircuit protection enabled
	bool psw_scd_en;
	
	// Shortcircuit protection threshold
	int psw_scd_tres;
	
	// Enable overtemperature protection
	bool t_psw_en;
	
	// Turn off power switch when MOSFET temperature is above this value
	float t_psw_max_mos;
	
	// Wait for init done before enabling power switch
	bool psw_wait_init;
} main_config_t;

// Default setting Overrides
#define HW_DEFAULT_ID				3

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// Other pins
#define PIN_SDA						21
#define PIN_SCL						20
#define PIN_ENABLE					2
#define PIN_PCHG_EN					8
#define PIN_COM_EN					9

// SD-card
#define SD_PIN_MOSI					0
#define SD_PIN_MISO					5
#define SD_PIN_SCK					4
#define SD_PIN_CS					10

// Parameters
#define HW_R_SHUNT					0.0002

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_VBMS16_H_ */
