/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_DRIVERS_BME280_BME280_IF_H_
#define MAIN_DRIVERS_BME280_BME280_IF_H_

// Functions
void bme280_if_init(int pin_sda, int pin_scl);
float bme280_if_get_hum(void);
float bme280_if_get_temp(void);
float bme280_if_get_pres(void);

#endif /* MAIN_DRIVERS_BME280_BME280_IF_H_ */
