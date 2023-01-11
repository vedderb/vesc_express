/*
	Copyright 2022 - 2023 Benjamin Vedder	benjamin@vedder.se

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
#include "driver/i2c.h"

#include "lispif.h"
#include "lispbm.h"

// Private variables
static float m_last_temp = 0.0;
static float m_last_hum = 0.0;

static uint8_t crc8(const uint8_t *data, uint8_t len)  {
	const uint8_t poly = 0x31;
	uint8_t crc = 0xFF;

	for (uint8_t j = len; j; --j) {
		crc ^= *data++;

		for (uint8_t i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
		}
	}

	return crc;
}

static void sht_task(void *arg) {
	for(;;) {
		uint8_t rxbuf[6];

		i2c_master_read_from_device(0, 0x70, rxbuf, 6, 1000 / portTICK_PERIOD_MS);

		if (rxbuf[2] == crc8(rxbuf, 2) && rxbuf[5] == crc8(rxbuf + 3, 2)) {
			uint16_t temp = (uint16_t)rxbuf[0] << 8 | (uint16_t)rxbuf[1];
			uint16_t hum = (uint16_t)rxbuf[3] << 8 | (uint16_t)rxbuf[4];

			m_last_temp = (float)temp / 65535.0 * 175.0 - 45.0;
			m_last_hum = (float)hum / 65535.0 * 100.0;
		} else {
			m_last_temp = 0.0;
			m_last_hum = 0.0;
		}

		// Start next measurement
		uint8_t txbuf[2];
		txbuf[0] = 0x78;
		txbuf[1] = 0x66;
		i2c_master_write_to_device(0, 0x70, txbuf, 2, 1000 / portTICK_PERIOD_MS);

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static lbm_value ext_hum_hum(lbm_value *args, lbm_uint argn) {
	return lbm_enc_float(m_last_hum);
}

static lbm_value ext_hum_temp(lbm_value *args, lbm_uint argn) {
	return lbm_enc_float(m_last_temp);
}

static void load_extensions(void) {
	lbm_add_extension("hum-hum", ext_hum_hum);
	lbm_add_extension("hum-temp", ext_hum_temp);
}

void hw_init(void) {
	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = SHTC3_SDA,
			.scl_io_num = SHTC3_SCL,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 100000,
	};

	i2c_param_config(0, &conf);
	i2c_driver_install(0, conf.mode, 0, 0, 0);

	xTaskCreatePinnedToCore(sht_task, "shtc3", 1024, NULL, 6, NULL, tskNO_AFFINITY);

	lispif_set_ext_load_callback(load_extensions);
}

float hw_hum_hum(void) {
	return m_last_hum;
}

float hw_hum_temp(void) {
	return m_last_temp;
}
