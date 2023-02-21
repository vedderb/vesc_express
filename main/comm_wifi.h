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

#ifndef MAIN_COMM_WIFI_H_
#define MAIN_COMM_WIFI_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_netif.h"

void comm_wifi_init(void);
esp_ip4_addr_t comm_wifi_get_ip(void);
esp_ip4_addr_t comm_wifi_get_ip_client(void);
bool comm_wifi_is_client_connected(void);
bool comm_wifi_is_connecting(void);
bool comm_wifi_is_connected(void);
void comm_wifi_send_packet_local(unsigned char *data, unsigned int len);
void comm_wifi_send_packet_hub(unsigned char *data, unsigned int len);
void comm_wifi_send_raw_local(unsigned char *buffer, unsigned int len);
void comm_wifi_send_raw_hub(unsigned char *buffer, unsigned int len);
void comm_wifi_disconnect(void);

#endif /* MAIN_COMM_WIFI_H_ */
