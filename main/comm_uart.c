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

#include "conf_general.h"
#include "commands.h"
#include "comm_uart.h"
#include "packet.h"
#include "driver/uart.h"

static PACKET_STATE_t packet_state;

static void rx_task(void *arg) {
	for (;;) {
		uint8_t buf[1];
		uart_read_bytes(UART_NUM, buf, 1, portMAX_DELAY);
		packet_process_byte(buf[0], &packet_state);
	}
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, comm_uart_send_packet);
}

static void send_packet_raw(unsigned char *buffer, unsigned int len) {
	uart_write_bytes(UART_NUM, buffer, len);
}

void comm_uart_init(void) {
	uart_config_t uart_config = {
			.baud_rate = UART_BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT,
	};

	uart_driver_install(UART_NUM, 512, 512, 0, 0, 0);
	uart_param_config(UART_NUM, &uart_config);
	uart_set_pin(UART_NUM, UART_TX, UART_RX, -1, -1);

	packet_init(send_packet_raw, process_packet, &packet_state);

	xTaskCreatePinnedToCore(rx_task, "usb_rx", 3072, NULL, 8, NULL, tskNO_AFFINITY);
}

void comm_uart_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, &packet_state);
}
