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
#include "sdkconfig.h"

void comm_ble_init(void);
bool comm_ble_is_connected(void);
int comm_ble_mtu_now(void);
void comm_ble_send_packet(unsigned char *data, unsigned int len);

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#include "host/ble_uuid.h"

typedef int (*ble_event_cb_t)(struct ble_gap_event *event, void *arg);

int comm_ble_host_init(const char *name, bool encrypted, ble_event_cb_t callback);
void comm_ble_host_start(void);
bool comm_ble_host_ready(void);
uint16_t comm_ble_conn_handle(void);
bool comm_ble_service_uuid_reserved(const ble_uuid_t *uuid);
int comm_ble_host_call(int (*callback)(void *arg), void *arg);
int comm_ble_host_advertise(struct ble_gap_adv_params *params, const uint8_t *adv_data, size_t adv_len,
	const uint8_t *scan_rsp_data, size_t scan_rsp_len);

#endif

#endif /* MAIN_COMM_BLE_H_ */
