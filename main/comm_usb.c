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

// Note: The usb_serial_jtag driver from master is copy-pasted here
// as it works much better. We do not want to be on master with
// the rest of the code.

typedef struct {
	uint32_t tx_buffer_size;                    /* Size of the buffer (in bytes) for the TX direction */
	uint32_t rx_buffer_size;                    /* Size of the buffer (in bytes) for the RX direction */
} usb_serial_jtag_driver_config_t;

typedef enum {
	FIFO_IDLE = 0,  /*!< Indicates the fifo is in idle state */
	FIFO_BUSY = 1,  /*!< Indicates the fifo is in busy state */
} fifo_status_t;

// The hardware buffer max size is 64
#define USB_SER_JTAG_ENDP_SIZE          (64)
#define USB_SER_JTAG_RX_MAX_SIZE        (64)

typedef struct {
	intr_handle_t intr_handle;          /*!< USB-SERIAL-JTAG interrupt handler */
	portMUX_TYPE spinlock;              /*!< Spinlock for usb_serial_jtag */
	_Atomic fifo_status_t fifo_status;  /*!< Record the status of fifo */

	// RX parameters
	RingbufHandle_t rx_ring_buf;        /*!< RX ring buffer handler */
	uint32_t rx_buf_size;               /*!< TX buffer size */
	uint8_t rx_data_buf[USB_SER_JTAG_ENDP_SIZE];            /*!< Data buffer to stash FIFO data */

	// TX parameters
	uint32_t tx_buf_size;               /*!< TX buffer size */
	RingbufHandle_t tx_ring_buf;        /*!< TX ring buffer handler */
	uint8_t tx_data_buf[USB_SER_JTAG_ENDP_SIZE];  /*!< Data buffer to stash TX FIFO data */
	size_t tx_stash_cnt;                          /*!< Number of stashed TX FIFO bytes */
} usb_serial_jtag_obj_t;

static usb_serial_jtag_obj_t *p_usb_serial_jtag_obj = NULL;

static size_t usb_serial_jtag_write_and_flush(const uint8_t *buf, uint32_t wr_len) {
	size_t size = usb_serial_jtag_ll_write_txfifo(buf, wr_len);
	usb_serial_jtag_ll_txfifo_flush();
	return size;
}

static void usb_serial_jtag_isr_handler_default(void *arg) {
	BaseType_t xTaskWoken = 0;
	uint32_t usbjtag_intr_status = 0;
	usbjtag_intr_status = usb_serial_jtag_ll_get_intsts_mask();

	if (usbjtag_intr_status & USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY) {
		// Interrupt tells us the host picked up the data we sent.
		// If we have more data, we can put it in the buffer and the host will pick that up next.
		// Send data in isr.
		// If the hardware fifo is available, write in it. Otherwise, do nothing.
		if (usb_serial_jtag_ll_txfifo_writable() == 1) {
			// We disable the interrupt here so that the interrupt won't be triggered if there is no data to send.

			size_t queued_size;
			uint8_t *queued_buff = NULL;
			bool is_stashed_data = false;
			if (p_usb_serial_jtag_obj->tx_stash_cnt != 0) {
				// Send stashed tx bytes before reading bytes from ring buffer
				queued_buff = p_usb_serial_jtag_obj->tx_data_buf;
				queued_size = p_usb_serial_jtag_obj->tx_stash_cnt;
				is_stashed_data = true;
			} else {
				// Max 64 data payload size in a single EndPoint
				queued_buff = (uint8_t *)xRingbufferReceiveUpToFromISR(p_usb_serial_jtag_obj->tx_ring_buf, &queued_size, USB_SER_JTAG_ENDP_SIZE);
			}

			usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);

			if (queued_buff != NULL) {

				// Although tx_queued_bytes may be larger than 0, we may have
				// interrupted before xRingbufferSend() was called.
				// Copy the queued buffer into the TX FIFO

				// On ringbuffer wrap-around the size can be 0 even though the buffer returned is not NULL
				if (queued_size > 0) {
					portENTER_CRITICAL_ISR(&p_usb_serial_jtag_obj->spinlock);
					atomic_store(&p_usb_serial_jtag_obj->fifo_status, FIFO_BUSY);
					uint32_t sent_size = usb_serial_jtag_write_and_flush(queued_buff, queued_size);
					portEXIT_CRITICAL_ISR(&p_usb_serial_jtag_obj->spinlock);

					if (sent_size < queued_size) {
						// Not all bytes could be sent at once; stash the unwritten bytes in a tx buffer
						// stash_size will not larger than USB_SER_JTAG_ENDP_SIZE because queued_size is got from xRingbufferReceiveUpToFromISR
						size_t stash_size = queued_size - sent_size;
						memcpy(p_usb_serial_jtag_obj->tx_data_buf, &queued_buff[sent_size], stash_size);
						p_usb_serial_jtag_obj->tx_stash_cnt = stash_size;
					} else {
						p_usb_serial_jtag_obj->tx_stash_cnt = 0;
						// assert if sent_size is larger than queued_size.
						assert(sent_size <= queued_size);
					}
				}
				if (is_stashed_data == false) {
					vRingbufferReturnItemFromISR(p_usb_serial_jtag_obj->tx_ring_buf, queued_buff, &xTaskWoken);
				}
			} else {
				// The last transmit may have sent a full EP worth of data. The host will interpret
				// this as a transaction that hasn't finished yet and keep the data in its internal
				// buffers rather than releasing it to the program listening on the CDC serial port.
				// We need to flush again in order to send a 0-byte packet that ends the transaction.
				usb_serial_jtag_ll_txfifo_flush();
				// Note that since this doesn't re-enable USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY, the
				// flush will not by itself cause this ISR to be called again.
			}
		} else {
			atomic_store(&p_usb_serial_jtag_obj->fifo_status, FIFO_IDLE);
			usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
		}
	}

	if (usbjtag_intr_status & USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT) {
		// read rx buffer(max length is 64), and send avaliable data to ringbuffer.
		// Ensure the rx buffer size is larger than RX_MAX_SIZE.
		usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
		uint32_t rx_fifo_len = usb_serial_jtag_ll_read_rxfifo(p_usb_serial_jtag_obj->rx_data_buf, USB_SER_JTAG_RX_MAX_SIZE);
		xRingbufferSendFromISR(p_usb_serial_jtag_obj->rx_ring_buf, p_usb_serial_jtag_obj->rx_data_buf, rx_fifo_len, &xTaskWoken);
	}

	if (xTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}

static esp_err_t a_usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *usb_serial_jtag_config) {
	esp_err_t err = ESP_OK;

	p_usb_serial_jtag_obj = (usb_serial_jtag_obj_t*) heap_caps_calloc(1, sizeof(usb_serial_jtag_obj_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	p_usb_serial_jtag_obj->rx_buf_size = usb_serial_jtag_config->rx_buffer_size;
	p_usb_serial_jtag_obj->tx_buf_size = usb_serial_jtag_config->tx_buffer_size;
	p_usb_serial_jtag_obj->tx_stash_cnt = 0;
	p_usb_serial_jtag_obj->spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

	if (p_usb_serial_jtag_obj == NULL) {
		err = ESP_ERR_NO_MEM;
		goto _exit;
	}

	p_usb_serial_jtag_obj->rx_ring_buf = xRingbufferCreate(p_usb_serial_jtag_obj->rx_buf_size, RINGBUF_TYPE_BYTEBUF);
	if (p_usb_serial_jtag_obj->rx_ring_buf == NULL) {
		err = ESP_ERR_NO_MEM;
		goto _exit;
	}

	p_usb_serial_jtag_obj->tx_ring_buf = xRingbufferCreate(usb_serial_jtag_config->tx_buffer_size, RINGBUF_TYPE_BYTEBUF);
	if (p_usb_serial_jtag_obj->rx_ring_buf == NULL) {
		err = ESP_ERR_NO_MEM;
		goto _exit;
	}

	atomic_store(&p_usb_serial_jtag_obj->fifo_status, FIFO_IDLE);

	usb_serial_jtag_ll_clr_intsts_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY |
			USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);
	usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY |
			USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT);

	err = esp_intr_alloc(ETS_USB_SERIAL_JTAG_INTR_SOURCE, 0, usb_serial_jtag_isr_handler_default, NULL, &p_usb_serial_jtag_obj->intr_handle);
	if (err != ESP_OK) {
		goto _exit;
	}
	return ESP_OK;

	_exit:

	return err;
}

static int a_usb_serial_jtag_read_bytes(void* buf, uint32_t length, TickType_t ticks_to_wait) {
	uint8_t *data = NULL;
	size_t data_read_len = 0;

	if (length == 0) {
		return 0;
	}

	// Recieve new data from ISR
	data = (uint8_t*) xRingbufferReceiveUpTo(p_usb_serial_jtag_obj->rx_ring_buf, &data_read_len, (TickType_t) ticks_to_wait, length);
	if (data == NULL) {
		// If there is no data received from ringbuffer, return 0 directly.
		return 0;
	}

	memcpy((uint8_t*)buf, data, data_read_len);
	vRingbufferReturnItem(p_usb_serial_jtag_obj->rx_ring_buf, data);
	data = NULL;

	return data_read_len;
}

static int a_usb_serial_jtag_write_bytes(const void* src, size_t size, TickType_t ticks_to_wait) {
	size_t sent_data = 0;
	BaseType_t result = pdTRUE;
	const uint8_t *buff = (const uint8_t *)src;
	if (p_usb_serial_jtag_obj->fifo_status == FIFO_IDLE) {
		portENTER_CRITICAL(&p_usb_serial_jtag_obj->spinlock);
		atomic_store(&p_usb_serial_jtag_obj->fifo_status, FIFO_BUSY);
		sent_data = usb_serial_jtag_write_and_flush(src, size);
		portEXIT_CRITICAL(&p_usb_serial_jtag_obj->spinlock);
	}

	// Blocking method, Sending data to ringbuffer, and handle the data in ISR.
	if (size - sent_data > 0) {
		result = xRingbufferSend(p_usb_serial_jtag_obj->tx_ring_buf, (void*)(buff + sent_data), size - sent_data, ticks_to_wait);
	} else {
		atomic_store(&p_usb_serial_jtag_obj->fifo_status, FIFO_IDLE);
	}
	usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
	return (result == pdFALSE) ? 0 : size;
}

static PACKET_STATE_t packet_state;

static void rx_task(void *arg) {
	for (;;) {
		uint8_t buf[1];
		a_usb_serial_jtag_read_bytes(buf, 1, portMAX_DELAY);
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

		unsigned int sent_now = a_usb_serial_jtag_write_bytes(buffer + sent, to_send, 10);
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
	a_usb_serial_jtag_driver_install(&usb_serial_jtag_config);

	packet_init(send_packet_raw, process_packet, &packet_state);

	xTaskCreatePinnedToCore(rx_task, "usb_rx", 3072, NULL, 8, NULL, tskNO_AFFINITY);
}

void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, &packet_state);
}
