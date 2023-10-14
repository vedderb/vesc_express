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

#ifndef MAIN_H_
#define MAIN_H_

#include "datatypes.h"
#include "hw.h"

// TODO: Move this to the config.
#define SETTING_CUSTOM_BLE true

#ifndef OVR_CONF_MAIN_CONFIG
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
} main_config_t;
#endif

// Init codes for the persistent storage. Change the config code when updating the config struct
// in a way that is not backwards compatible.
#ifndef VAR_INIT_CODE
#define VAR_INIT_CODE			259763459
#endif

// Backup data that is retained between boots and firmware updates. When adding new
// entries, put them at the end.
typedef struct {
	// Store CAN-related settings separate from config as well. This is done in order
	// to retain the CAN-settings after doing distributed firmware updates that change
	// the main config signature.
	uint32_t controller_id_init_flag;
	uint16_t controller_id;
	uint32_t can_baud_rate_init_flag;
	CAN_BAUD can_baud_rate;

	// Main configuration structure
	uint32_t config_init_flag;
	main_config_t config;

	// Pad just in case as flash_helper_write_data rounds length down to
	// closest multiple of 8.
	volatile uint32_t pad1;
	volatile uint32_t pad2;
} backup_data;

// Global variables
extern volatile backup_data backup;

// Functions
void main_store_backup_data(void);
bool main_init_done(void);
void main_wait_until_init_done(void);

#endif /* MAIN_H_ */
