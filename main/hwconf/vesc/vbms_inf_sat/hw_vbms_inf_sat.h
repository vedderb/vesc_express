/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_VESC_VBMS_INF_SAT_H_
#define MAIN_HWCONF_VESC_VBMS_INF_SAT_H_

#include "adc.h"
#include "driver/gpio.h"
#include "datatypes.h"

#define HW_NAME						"VBMS-INF-SAT"
#define HW_TARGET                   "esp32c3"

#define HW_EARLY_LBM_INIT
#define HW_NO_UART
#define HW_INIT_HOOK()				hw_init()
//#define HW_POST_LISPIF_HOOK()		vTaskDelay(200);

#ifndef CONF_WIFI_MODE
#define CONF_WIFI_MODE 0
#endif

// Bluetooth Mode
#ifndef CONF_BLE_MODE
#define CONF_BLE_MODE 0
#endif

// Other pins
#define PIN_SDA						4
#define PIN_SCL						5
#define PIN_ENABLE					2
#define PIN_COM_EN					9

// Parameters
#define HW_R_SHUNT					0.0002

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_VBMS_INF_SAT_H_ */
