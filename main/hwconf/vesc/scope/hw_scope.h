/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_VESC_SCOPE_H_
#define MAIN_HWCONF_VESC_SCOPE_H_

#include "driver/gpio.h"
#include "adc.h"

#define HW_NAME						"VL Scope"

#define HW_NO_UART

#define HW_INIT_HOOK()				hw_init()

// CAN
#define CAN_TX_GPIO_NUM				0
#define CAN_RX_GPIO_NUM				1

// SD-card
#define SD_PIN_MOSI					5
#define SD_PIN_MISO					10
#define SD_PIN_SCK					4
#define SD_PIN_CS					9

// Display
#define DISP_SD0					5
#define DISP_CLK					4
#define DISP_CS						2
#define DISP_RESET					3
#define DISP_DC						8

// Buttons
#define PIN_BT1						6
#define PIN_BT2						7

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_SCOPE_H_ */
