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

#include "commands.h"
#include "comm_usb.h"
#include "packet.h"

#include <string.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "hal/usb_serial_jtag_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_intr_alloc.h"
#include "soc/periph_defs.h"
#include "soc/soc_caps.h"
#include "esp_private/periph_ctrl.h"
#include "driver/usb_serial_jtag.h"

static PACKET_STATE_t packet_state;

static void rx_task(void *arg) {
	for (;;) {
		uint8_t buf[1];
		usb_serial_jtag_read_bytes(buf, 1, portMAX_DELAY);
		packet_process_byte(buf[0], &packet_state);
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_usb_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	unsigned int sent = 0;
	int fail_cnt = 0;

	while (sent < len) {
		int to_send = len - sent;
		if (to_send > 150) {
			to_send = 150;
		}

		unsigned int sent_now = usb_serial_jtag_write_bytes(buffer + sent, to_send, 10);
		sent += sent_now;

		if (sent_now == 0) {
			fail_cnt++;
		} else {
			fail_cnt = 0;
		}

		if (fail_cnt >= 3) {
			break;
		}
	}
}

void comm_usb_init(void) {
	#if CONFIG_IDF_TARGET_ESP32
		return;
	#endif


	usb_serial_jtag_driver_config_t usb_serial_jtag_config;
	usb_serial_jtag_config.rx_buffer_size = 1024;
	usb_serial_jtag_config.tx_buffer_size = 256;
	usb_serial_jtag_driver_install(&usb_serial_jtag_config);

	packet_init(send_packet_raw, process_packet, &packet_state);

	xTaskCreatePinnedToCore(rx_task, "usb_rx", 3072, NULL, 8, NULL, tskNO_AFFINITY);
}

void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, &packet_state);
}
