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

#include "mempools.h"
#include "packet.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Private types
typedef struct {
	volatile bool is_taken;
	main_config_t conf;
} conf_container_t;

// Private variables
static conf_container_t m_confs[MEMPOOLS_CONF_NUM] = {{0}};
static int m_conf_highest = 0;
static uint8_t packet_buffer[PACKET_MAX_PL_LEN];
static SemaphoreHandle_t packet_buffer_mutex;

void mempools_init(void) {
	packet_buffer_mutex = xSemaphoreCreateMutex();
}

main_config_t *mempools_alloc_conf(void) {
	for (int i = 0;i < MEMPOOLS_CONF_NUM;i++) {
		if (i > m_conf_highest) {
			m_conf_highest = i;
		}
		if (!m_confs[i].is_taken) {
			m_confs[i].is_taken = true;
			return &m_confs[i].conf;
		}
	}

	m_conf_highest++;

	return 0;
}

void mempools_free_conf(main_config_t *conf) {
	for (int i = 0;i < MEMPOOLS_CONF_NUM;i++) {
		if (&m_confs[i].conf == conf) {
			m_confs[i].is_taken = false;
			return;
		}
	}
}

int mempools_conf_highest(void) {
	return m_conf_highest;
}

int mempools_conf_allocated_num(void) {
	int res = 0;
	for (int i = 0;i < MEMPOOLS_CONF_NUM;i++) {
		if (m_confs[i].is_taken) {
			res++;
		}
	}
	return res;
}

uint8_t *mempools_get_packet_buffer(void) {
	xSemaphoreTake(packet_buffer_mutex, portMAX_DELAY);
	return packet_buffer;
}

void mempools_free_packet_buffer(uint8_t *buffer) {
	if (buffer == packet_buffer) {
		xSemaphoreGive(packet_buffer_mutex);
	}
}
