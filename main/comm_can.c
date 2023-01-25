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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "datatypes.h"
#include "buffer.h"
#include "driver/twai.h"
#include "comm_can.h"
#include "datatypes.h"
#include "conf_general.h"
#include "main.h"
#include "crc.h"
#include "packet.h"
#include "commands.h"
#include "nmea.h"
#include "lispif.h"

#include <string.h>

#define RX_BUFFER_NUM				3
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN

// For double precision literals
#define D(x) 						((double)x##L)

static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_NUM, CAN_RX_GPIO_NUM, TWAI_MODE_NORMAL);

static SemaphoreHandle_t ping_sem;
static SemaphoreHandle_t send_mutex;
static volatile HW_TYPE ping_hw_last = HW_TYPE_VESC;
uint8_t rx_buffer[RX_BUFFER_NUM][RX_BUFFER_SIZE];
int rx_buffer_offset[RX_BUFFER_NUM];
static unsigned int rx_buffer_last_id;

// Private functions
static void update_baud(CAN_BAUD baudrate);

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(rx_buffer_last_id, data, len, 1);
}

static void decode_msg(uint32_t eid, uint8_t *data8, int len, bool is_replaced) {
	int32_t ind = 0;
	uint8_t crc_low;
	uint8_t crc_high;
	uint8_t commands_send;

	uint8_t id = eid & 0xFF;
	CAN_PACKET_ID cmd = eid >> 8;

	if (id == 255 || id == backup.config.controller_id) {
		switch (cmd) {
		case CAN_PACKET_FILL_RX_BUFFER: {
			int buf_ind = 0;
			int offset = data8[0];
			data8++;
			len--;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			memcpy(rx_buffer[buf_ind] + offset, data8, len);
			rx_buffer_offset[buf_ind] += len;
		} break;

		case CAN_PACKET_FILL_RX_BUFFER_LONG: {
			int buf_ind = 0;
			int offset = (int)data8[0] << 8;
			offset |= data8[1];
			data8 += 2;
			len -= 2;

			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == offset ) {
					buf_ind = i;
					break;
				}
			}

			if ((offset + len) <= RX_BUFFER_SIZE) {
				memcpy(rx_buffer[buf_ind] + offset, data8, len);
				rx_buffer_offset[buf_ind] += len;
			}
		} break;

		case CAN_PACKET_PROCESS_RX_BUFFER: {
			ind = 0;
			unsigned int last_id = data8[ind++];
			commands_send = data8[ind++];

			if (commands_send == 0) {
				rx_buffer_last_id = last_id;
			}

			int rxbuf_len = (int)data8[ind++] << 8;
			rxbuf_len |= (int)data8[ind++];

			if (rxbuf_len > RX_BUFFER_SIZE) {
				break;
			}

			int buf_ind = -1;
			for (int i = 0; i < RX_BUFFER_NUM;i++) {
				if ((rx_buffer_offset[i]) == rxbuf_len ) {
					buf_ind = i;
					break;
				}
			}

			// Something is wrong, reset all buffers
			if (buf_ind < 0) {
				for (int i = 0; i < RX_BUFFER_NUM;i++) {
					rx_buffer_offset[i] = 0;
				}
				break;
			}

			rx_buffer_offset[buf_ind] = 0;

			crc_high = data8[ind++];
			crc_low = data8[ind++];

			if (crc16(rx_buffer[buf_ind], rxbuf_len)
					== ((unsigned short) crc_high << 8
							| (unsigned short) crc_low)) {

				if (is_replaced) {
					if (rx_buffer[buf_ind][0] == COMM_JUMP_TO_BOOTLOADER ||
							rx_buffer[buf_ind][0] == COMM_ERASE_NEW_APP ||
							rx_buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA ||
							rx_buffer[buf_ind][0] == COMM_WRITE_NEW_APP_DATA_LZO ||
							rx_buffer[buf_ind][0] == COMM_ERASE_BOOTLOADER) {
						break;
					}
				}

				switch (commands_send) {
				case 0:
					commands_process_packet(rx_buffer[buf_ind], rxbuf_len, send_packet_wrapper);
					break;
				case 1:
					commands_send_packet_can_last(rx_buffer[buf_ind], rxbuf_len);
					break;
				case 2:
					commands_process_packet(rx_buffer[buf_ind], rxbuf_len, 0);
					break;
				default:
					break;
				}
			}
		} break;

		case CAN_PACKET_PROCESS_SHORT_BUFFER:
			ind = 0;
			rx_buffer_last_id = data8[ind++];
			commands_send = data8[ind++];

			if (is_replaced) {
				if (data8[ind] == COMM_JUMP_TO_BOOTLOADER ||
						data8[ind] == COMM_ERASE_NEW_APP ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA ||
						data8[ind] == COMM_WRITE_NEW_APP_DATA_LZO ||
						data8[ind] == COMM_ERASE_BOOTLOADER) {
					break;
				}
			}

			switch (commands_send) {
			case 0:
				commands_process_packet(data8 + ind, len - ind, send_packet_wrapper);
				break;
			case 1:
				commands_send_packet(data8 + ind, len - ind);
				break;
			case 2:
				commands_process_packet(data8 + ind, len - ind, 0);
				break;
			default:
				break;
			}
			break;

			case CAN_PACKET_PING: {
				uint8_t buffer[2];
				buffer[0] = backup.config.controller_id;
				buffer[1] = HW_TYPE_CUSTOM_MODULE;
				comm_can_transmit_eid(data8[0] | ((uint32_t)CAN_PACKET_PONG << 8), buffer, 2);
			} break;

			case CAN_PACKET_PONG:
				// data8[0]; // Sender ID
				xSemaphoreGive(ping_sem);
				if (len >= 2) {
					ping_hw_last = data8[1];
				} else {
					ping_hw_last = HW_TYPE_VESC_BMS;
				}
				break;

			default:
				break;
		}
	}
}

#define RXBUF_LEN			100
static twai_message_t rx_buf[RXBUF_LEN];
static volatile int rx_write = 0;
static volatile int rx_read = 0;
static SemaphoreHandle_t proc_sem;

static void rx_task(void *arg) {
	twai_message_t rx_message;

	for (;;) {
		esp_err_t res = twai_receive(&rx_message, 10 / portTICK_PERIOD_MS);

		if (res == ESP_OK) {
			rx_buf[rx_write] = rx_message;
			rx_write++;
			if (rx_write >= RXBUF_LEN) {
				rx_write = 0;
			}

			xSemaphoreGive(proc_sem);
		}
	}

	vTaskDelete(NULL);
}

static void process_task(void *arg) {
	for (;;) {
		xSemaphoreTake(proc_sem, 10 / portTICK_PERIOD_MS);

		while (rx_read != rx_write) {
			twai_message_t *msg = &rx_buf[rx_read];
			rx_read++;
			if (rx_read >= RXBUF_LEN) {
				rx_read = 0;
			}

			lispif_process_can(msg->identifier, msg->data, msg->data_length_code, msg->extd);

			if (msg->extd) {
				decode_msg(msg->identifier, msg->data, msg->data_length_code, false);
			}
		}
	}
}

static void status_task(void *arg) {
	int gga_cnt_last = 0;
	int rmc_cnt_last = 0;

	for (;;) {
		int rate = backup.config.can_status_rate_hz;

		if (rate < 1) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
			continue;
		}

#ifdef HW_CAN_STATUS_ADC0
		{
			int32_t send_index = 0;
			uint8_t buffer[8];

			buffer_append_float16(buffer, HW_CAN_STATUS_ADC0, 1e2, &send_index);
			buffer_append_float16(buffer, HW_CAN_STATUS_ADC1, 1e2, &send_index);
			buffer_append_float16(buffer, HW_CAN_STATUS_ADC2, 1e2, &send_index);
			buffer_append_float16(buffer, HW_CAN_STATUS_ADC3, 1e2, &send_index);
			comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_IO_BOARD_ADC_1_TO_4 << 8), buffer, send_index);
		}
#endif

		{ // GNSS
			nmea_state_t *s = nmea_get_state();

			bool date_valid = true;
			if (s->rmc.yy < 0 || s->rmc.mo < 0 || s->rmc.dd < 0 ||
					s->rmc.hh < 0 || s->rmc.mm < 0 || s->rmc.ss < 0) {
				date_valid = false;
			}

			bool gga_updated = false;
			if (s->gga_cnt != gga_cnt_last) {
				gga_updated = true;
				gga_cnt_last = s->gga_cnt;
			}

			bool rmc_updated = false;
			if (s->rmc_cnt != rmc_cnt_last) {
				rmc_updated = true;
				rmc_cnt_last = s->rmc_cnt;
			}

			if (date_valid && rmc_updated) {
				int32_t send_index = 0;
				uint8_t buffer[8];
				buffer_append_int32(buffer, s->gga.ms_today, &send_index);
				buffer_append_int16(buffer, s->rmc.yy, &send_index);
				buffer[send_index++] = s->rmc.mo;
				buffer[send_index++] = s->rmc.dd;
				comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_GNSS_TIME << 8), buffer, send_index);
			}

			if (gga_updated) {
				// Lat
				int32_t send_index = 0;
				uint8_t buffer[8];
				buffer_append_double64(buffer, s->gga.lat, D(1e16), &send_index);
				comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_GNSS_LAT << 8), buffer, send_index);

				// Lon
				send_index = 0;
				buffer_append_double64(buffer, s->gga.lon, D(1e16), &send_index);
				comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_GNSS_LON << 8), buffer, send_index);

				// Alt, speed, hdop
				send_index = 0;
				buffer_append_float32_auto(buffer, s->gga.height, &send_index);
				buffer_append_float16(buffer, s->rmc.speed, 1.0e2, &send_index);
				buffer_append_float16(buffer, s->gga.h_dop, 1.0e2, &send_index);
				comm_can_transmit_eid(backup.config.controller_id | ((uint32_t)CAN_PACKET_GNSS_ALT_SPEED_HDOP << 8), buffer, send_index);
			}
		}

		vTaskDelay(configTICK_RATE_HZ / rate);
	}
}

void comm_can_init(void) {
	ping_sem = xSemaphoreCreateBinary();
	proc_sem = xSemaphoreCreateBinary();
	send_mutex = xSemaphoreCreateMutex();

	update_baud(backup.config.can_baud_rate);

	twai_driver_install(&g_config, &t_config, &f_config);
	twai_start();

	xTaskCreatePinnedToCore(status_task, "can_status", 1024, NULL, 7, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(rx_task, "can_rx", 1024, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(process_task, "can_proc", 4096, NULL, 8, NULL, tskNO_AFFINITY);
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	twai_message_t tx_msg = {0};
	tx_msg.extd = 1;
	tx_msg.identifier = id;

	memcpy(tx_msg.data, data, len);
	tx_msg.data_length_code = len;

	xSemaphoreTake(send_mutex, portMAX_DELAY);
	if (twai_transmit(&tx_msg, 5 / portTICK_PERIOD_MS) != ESP_OK) {
		twai_stop();
		twai_initiate_recovery();
		twai_start();
	}
	xSemaphoreGive(send_mutex);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	twai_message_t tx_msg = {0};
	tx_msg.extd = 0;
	tx_msg.identifier = id;

	memcpy(tx_msg.data, data, len);
	tx_msg.data_length_code = len;

	xSemaphoreTake(send_mutex, portMAX_DELAY);
	if (twai_transmit(&tx_msg, 5 / portTICK_PERIOD_MS) != ESP_OK) {
		twai_stop();
		twai_initiate_recovery();
		twai_start();
	}
	xSemaphoreGive(send_mutex);
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * 0: Packet goes to commands_process_packet of receiver
 * 1: Packet goes to commands_send_packet of receiver
 * 2: Packet goes to commands_process and send function is set to null
 *    so that no reply is sent back.
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = backup.config.controller_id;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id |
					((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = backup.config.controller_id;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id |
				((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

/**
 * Check if a VESC on the CAN-bus responds.
 *
 * @param controller_id
 * The ID of the VESC.
 *
 * @param hw_type
 * The hardware type of the CAN device.
 *
 * @return
 * True for success, false otherwise.
 */
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type) {
	uint8_t buffer[1];
	buffer[0] = backup.config.controller_id;
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PING << 8), buffer, 1);

	bool ret = xSemaphoreTake(ping_sem, 10 / portTICK_PERIOD_MS) == pdTRUE;

	if (ret) {
		if (hw_type) {
			*hw_type = ping_hw_last;
		}
	}

	return ret;
}

static void update_baud(CAN_BAUD baudrate) {
	switch (baudrate) {
	case CAN_BAUD_125K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_125KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_250K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_250KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_500K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_500KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_1M: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_1MBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_10K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_10KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_20K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_20KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_50K: {
		twai_timing_config_t t_config2 = TWAI_TIMING_CONFIG_50KBITS();
		t_config = t_config2;
	} break;

	case CAN_BAUD_75K: {
		// Invalid
	} break;

	default:
		break;
	}
}
