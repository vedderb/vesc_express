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

#include "hwi2c.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>

typedef struct i2c_device {
	struct i2c_device *next;
	i2c_master_dev_handle_t handle;
	uint8_t addr;
} i2c_device_t;

static struct {
	i2c_master_bus_handle_t handle;
	i2c_device_t *devices;
	uint32_t speed;
	int sda;
	int scl;
} i2c_buses[SOC_I2C_NUM];
static StaticSemaphore_t i2c_lock_buffer;
static SemaphoreHandle_t i2c_lock;
static portMUX_TYPE i2c_init_lock = portMUX_INITIALIZER_UNLOCKED;

static void lock_i2c(void) {
	taskENTER_CRITICAL(&i2c_init_lock);
	if (!i2c_lock) {
		i2c_lock = xSemaphoreCreateMutexStatic(&i2c_lock_buffer);
	}
	taskEXIT_CRITICAL(&i2c_init_lock);
	xSemaphoreTake(i2c_lock, portMAX_DELAY);
}

static void clear_i2c_devices(i2c_port_num_t port) {
	while (i2c_buses[port].devices) {
		i2c_device_t *dev = i2c_buses[port].devices;
		i2c_buses[port].devices = dev->next;
		i2c_master_bus_rm_device(dev->handle);
		free(dev);
	}
}

i2c_master_bus_handle_t hwi2c_bus(i2c_port_num_t port) {
	return port >= 0 && port < SOC_I2C_NUM ? i2c_buses[port].handle : NULL;
}

esp_err_t hwi2c_init(i2c_port_num_t port, int sda, int scl, uint32_t speed, bool pullup) {
	if (port < 0 || port >= SOC_I2C_NUM || !speed) {
		return ESP_ERR_INVALID_ARG;
	}

	lock_i2c();
	esp_err_t res = ESP_OK;
	if (i2c_buses[port].handle) {
		if (i2c_buses[port].sda != sda || i2c_buses[port].scl != scl) {
			clear_i2c_devices(port);
			res = i2c_del_master_bus(i2c_buses[port].handle);
			if (res == ESP_OK) {
				i2c_buses[port].handle = NULL;
			}
		} else if (i2c_buses[port].speed != speed) {
			clear_i2c_devices(port);
			i2c_buses[port].speed = speed;
		}
	}
	if (res == ESP_OK && !i2c_buses[port].handle) {
		i2c_master_bus_config_t config = {
			.i2c_port = port,
			.sda_io_num = sda,
			.scl_io_num = scl,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.flags.enable_internal_pullup = pullup,
		};
		res = i2c_new_master_bus(&config, &i2c_buses[port].handle);
		if (res == ESP_OK) {
			i2c_buses[port].sda = sda;
			i2c_buses[port].scl = scl;
			i2c_buses[port].speed = speed;
		}
	}
	xSemaphoreGive(i2c_lock);
	return res;
}

esp_err_t hwi2c_stop(i2c_port_num_t port) {
	if (port < 0 || port >= SOC_I2C_NUM) {
		return ESP_ERR_INVALID_ARG;
	}

	lock_i2c();
	esp_err_t res = ESP_OK;
	if (i2c_buses[port].handle) {
		clear_i2c_devices(port);
		res = i2c_del_master_bus(i2c_buses[port].handle);
		if (res == ESP_OK) {
			i2c_buses[port].handle = NULL;
		}
	}
	xSemaphoreGive(i2c_lock);
	return res;
}

esp_err_t hwi2c_tx_rx(i2c_port_num_t port, uint8_t addr, const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen, int timeout_ms) {
	if (port < 0 || port >= SOC_I2C_NUM || addr > 0x7F || (txlen && !tx) || (rxlen && !rx)) {
		return ESP_ERR_INVALID_ARG;
	}

	lock_i2c();
	esp_err_t res = ESP_ERR_INVALID_STATE;
	if (!i2c_buses[port].handle) {
		goto done;
	}
	if (!txlen && !rxlen) {
		res = i2c_master_probe(i2c_buses[port].handle, addr, timeout_ms);
		goto done;
	}
	i2c_device_t *dev = i2c_buses[port].devices;
	while (dev && dev->addr != addr) {
		dev = dev->next;
	}
	if (!dev) {
		dev = calloc(1, sizeof(*dev));
		if (!dev) {
			res = ESP_ERR_NO_MEM;
			goto done;
		}
		i2c_device_config_t config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = addr,
			.scl_speed_hz = i2c_buses[port].speed,
		};
		res = i2c_master_bus_add_device(i2c_buses[port].handle, &config, &dev->handle);
		if (res != ESP_OK) {
			free(dev);
			goto done;
		}
		dev->addr = addr;
		dev->next = i2c_buses[port].devices;
		i2c_buses[port].devices = dev;
	}
	if (txlen && rxlen) {
		res = i2c_master_transmit_receive(dev->handle, tx, txlen, rx, rxlen, timeout_ms);
	} else if (rxlen) {
		res = i2c_master_receive(dev->handle, rx, rxlen, timeout_ms);
	} else {
		res = i2c_master_transmit(dev->handle, tx, txlen, timeout_ms);
	}

done:
	xSemaphoreGive(i2c_lock);
	return res;
}
