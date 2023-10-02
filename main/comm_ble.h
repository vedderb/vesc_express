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

#ifndef MAIN_COMM_BLE_H_
#define MAIN_COMM_BLE_H_

#include <stdint.h>
#include <stdbool.h>

/** GATT server. */
#define GATT_SVR_SVC_ALERT_UUID               0x1811
#define GATT_SVR_CHR_SUP_NEW_ALERT_CAT_UUID   0x2A47
#define GATT_SVR_CHR_NEW_ALERT                0x2A46
#define GATT_SVR_CHR_SUP_UNR_ALERT_CAT_UUID   0x2A48
#define GATT_SVR_CHR_UNR_ALERT_STAT_UUID      0x2A45
#define GATT_SVR_CHR_ALERT_NOT_CTRL_PT        0x2A44

void comm_ble_init(void);
bool comm_ble_is_connected();
int comm_ble_mtu_now(void);
const char *comm_ble_get_error_message();
const char *comm_ble_get_message();
void comm_ble_print_chr();
void comm_ble_send_packet(unsigned char *data, unsigned int len);

// TODO: remove this
typedef void (*send_func_t)(unsigned char*,unsigned int);
void comm_ble_store_curr_send_func();
send_func_t comm_ble_get_stored_send_func();

#endif /* MAIN_COMM_BLE_H_ */
