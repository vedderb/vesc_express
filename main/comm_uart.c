/*
	Copyright 2022 - 2024 Benjamin Vedder	benjamin@vedder.se

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
#include <string.h>

typedef struct {
	int uart_num;
	PACKET_STATE_t packet_state;
	volatile bool should_stop;
	volatile bool is_running;
} uart_state;

static uart_state *m_state[UART_NUM_MAX] = {0};

static void rx_task(void *arg) {
	uart_state *state = (uart_state*)arg;

	state->is_running = true;

	while (!state->should_stop) {
		uint8_t buf[1];
		int bytes = uart_read_bytes(state->uart_num, buf, 1, 3);
		if (bytes > 0) {
			packet_process_byte(buf[0], &(state->packet_state));
		}

		// Check if this uart has been stopped externally
		if (!uart_is_driver_installed(state->uart_num)) {
			m_state[state->uart_num] = NULL;
			free(state);
			state = NULL;
			break;
		}
	}

	if (state) {
		state->is_running = false;
	}

	vTaskDelete(NULL);
}

static void send_packet_u0(unsigned char *data, unsigned int len) {
	comm_uart_send_packet(data, len, 0);
}

static void send_packet_u1(unsigned char *data, unsigned int len) {
	comm_uart_send_packet(data, len, 1);
}

static void process_packet_u0(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, send_packet_u0);
}

static void process_packet_u1(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, send_packet_u1);
}

static void send_packet_raw_u0(unsigned char *buffer, unsigned int len) {
	uart_write_bytes(0, buffer, len);
}

static void send_packet_raw_u1(unsigned char *buffer, unsigned int len) {
	uart_write_bytes(1, buffer, len);
}

bool comm_uart_init(int pin_tx, int pin_rx, int uart_num, int baudrate) {
	if (uart_num >= 0 && uart_num >= UART_NUM_MAX) {
		return false;
	}

	comm_uart_stop(uart_num);

	uart_state *state;
	state = malloc(sizeof(uart_state));
	if (!state) {
		return false;
	}
	memset(state, 0, sizeof(uart_state));
	state->uart_num = uart_num;

	uart_config_t uart_config = {
			.baud_rate = baudrate,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT,
	};

	uart_driver_install(uart_num, 512, 512, 0, 0, 0);
	uart_param_config(uart_num, &uart_config);
	uart_set_pin(uart_num, pin_tx, pin_rx, -1, -1);

	if (uart_num == 0) {
		packet_init(send_packet_raw_u0, process_packet_u0, &(state->packet_state));
	} else {
		packet_init(send_packet_raw_u1, process_packet_u1, &(state->packet_state));
	}

	xTaskCreatePinnedToCore(rx_task, "uart_rx", 3072, state, 8, NULL, tskNO_AFFINITY);
	m_state[uart_num] = state;

	return true;
}

void comm_uart_stop(int uart_num) {
	if (uart_num >= 0 && uart_num >= UART_NUM_MAX) {
		return;
	}

	uart_state *state = m_state[uart_num];
	if (state) {
		m_state[uart_num] = NULL;

		state->should_stop = true;
		while (state->is_running) {
			vTaskDelay(1);
		}

		vTaskDelay(1);
		free(state);
	}

	if (uart_is_driver_installed(uart_num)) {
		uart_driver_delete(uart_num);
	}
}

void comm_uart_send_packet(unsigned char *data, unsigned int len, int uart_num) {
	if (uart_num < 0 || uart_num >= UART_NUM_MAX || m_state[uart_num] == NULL) {
		return;
	}

	packet_send_packet(data, len, &(m_state[uart_num]->packet_state));
}
