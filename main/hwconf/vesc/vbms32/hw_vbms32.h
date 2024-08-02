/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_VESC_VBMS32_H_
#define MAIN_HWCONF_VESC_VBMS32_H_

/*
 * TODO V1:
 * * [OK] Fix precharge supply. 12V boost from 5V is an option. E.g. MT3608.
 * * [OK] Connect temp-pin properly so that bq can exit sleep mode (not needed?)
 * * [OK] RST_SHUT should have pull-down when not used
 * * [OK] 10k pull-up on i2c
 * * [OK] SRP and SRN to Cell0 when not used
 * * SRP should be positive when charging (Was OK, wrong now...)
 * * [OK] How to deal with I2C when only having the lower BQ connected?
 */

/*
 * TODO V2:
 * Swap SRP and SRN again
 * Temperature sensor under MOSFETs
 */

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME						"VBMS32"

/*
 * PCB-versions
 *
 * 1: First prototype, PCB says 1.0
 * 2: Second prototype, PCB says 1.1
 *
 */
#define PCB_VERSION					2

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_INIT_HOOK()				hw_init()
//#define HW_POST_LISPIF_HOOK()		vTaskDelay(200);

// Configuration overrides
#define OVR_CONF_PARSER_C			"vbms32_confparser.c"
#define OVR_CONF_PARSER_H			"vbms32_confparser.h"
#define OVR_CONF_XML_C				"vbms32_confxml.c"
#define OVR_CONF_XML_H				"vbms32_confxml.h"
#define OVR_CONF_DEFAULT			"vbms32_conf_default.h"
#define OVR_CONF_SERIALIZE			vbms32_confparser_serialize_main_config_t
#define OVR_CONF_DESERIALIZE		vbms32_confparser_deserialize_main_config_t
#define OVR_CONF_SET_DEFAULTS		vbms32_confparser_set_defaults_main_config_t
#define OVR_CONF_MAIN_CONFIG
#define VAR_INIT_CODE				259763459

typedef enum {
	BALANCE_MODE_DISABLED = 0,
	BALANCE_MODE_CHARGING_ONLY,
	BALANCE_MODE_DURING_AND_AFTER_CHARGING,
	BALANCE_MODE_ALWAYS
} BMS_BALANCE_MODE;

typedef enum {
	I_MEASURE_MODE_BMS = 0,
	I_MEASURE_MODE_VESC
} I_MEASURE_MODE;

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

	// Cells on first balance IC
	int cells_ic1;

	// Cells on second balance IC
	int cells_ic2;

	// Cell balancing mode
	BMS_BALANCE_MODE balance_mode;

	// Maximum simultaneous balancing channels
	int max_bal_ch;

	// Distributed balancing
	bool dist_bal;

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

	// Current measurement mode
	I_MEASURE_MODE i_measure_mode;

	// Reset sleep timeout to this value at events that prevent sleeping
	int sleep_timeout_reset_ms;

	// Stop charging when the charge current goes below this value
	float min_charge_current;

	// Maximum allowed charging current
	float max_charge_current;

	// Filter constant for SoC filter
	float soc_filter_const;

	// Start limiting the number of balancing channels at this temperature
	float t_bal_lim_start;

	// Disable all balancing channels above this temperature
	float t_bal_lim_end;

	// Only allow charging when the cell temperature is above this value
	float t_charge_min;

	// Enable temperature monitoring during charging
	bool t_charge_mon_en;
} main_config_t;

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// Other pins
#define PIN_SDA						21
#define PIN_SCL						20
#define PIN_ENABLE					2
#define PIN_OUT_EN					4
#define PIN_CHG_EN					5
#define PIN_PCHG_EN					8
#define PIN_COM_EN					9
#define PIN_PSW_EN					10

// ADC
#define HW_ADC_CH0					ADC1_CHANNEL_0 // DIV_CHG
#define HW_ADC_CH1					ADC1_CHANNEL_1 // DIV_OUT

// Parameters
#define HW_R_SHUNT					0.0002

// Macros
#define HW_GET_VOUT()				((adc_get_voltage(ADC1_CHANNEL_1) * (220.0e3 + 4.7e3)) / 4.7e3)
#define HW_GET_VCHG()				((adc_get_voltage(ADC1_CHANNEL_0) * (220.0e3 + 4.7e3)) / 4.7e3)

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_VBMS32_H_ */
