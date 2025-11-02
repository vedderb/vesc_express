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

/* ESP32C3 Supermini
   - https://www.fambach.net/esp32-c3-super-mini-board/
   - https://forum.esk8.news/t/usb-c-to-vesc-can-forward-adapter-cheap-and-diy/79789
   */

#ifndef MAIN_HWCONF_C3_SUPERMINI_H_
#define MAIN_HWCONF_C3_SUPERMINI_H_

#include "driver/gpio.h"

#define HW_NAME						"C3 Supermini"
#define HW_DEFAULT_ID				2

#define HW_INIT_HOOK()				hw_init()

// LEDs
// Blue is shared (GPIO8/SCK/D8), Red is not controllable (Power LED)
#define LED_BLUE_PIN				8

// CAN
#define CAN_TX_GPIO_NUM				1
#define CAN_RX_GPIO_NUM				0

// UART
#define HW_NO_UART

// Functions
void hw_init(void);

#endif
