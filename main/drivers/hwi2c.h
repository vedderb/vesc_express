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

#ifndef MAIN_DRIVERS_HWI2C_H_
#define MAIN_DRIVERS_HWI2C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"

esp_err_t hwi2c_init(i2c_port_num_t port, int sda, int scl, uint32_t speed, bool pullup);
esp_err_t hwi2c_stop(i2c_port_num_t port);
i2c_master_bus_handle_t hwi2c_bus(i2c_port_num_t port);
esp_err_t hwi2c_tx_rx(i2c_port_num_t port, uint8_t addr, const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen, int timeout_ms);

#endif /* MAIN_DRIVERS_HWI2C_H_ */
