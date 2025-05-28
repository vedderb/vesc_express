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

#ifndef MAIN_HWCONF_LIFAN_H_
#define MAIN_HWCONF_LIFAN_H_

#include "driver/gpio.h"

#define HW_NAME						"Lifan Reg"
#define HW_TARGET					"esp32c3"
#define HW_DEFAULT_ID				38
#define HW_NO_UART

#define HW_INIT_HOOK()				hw_init()

// CAN
#define CAN_TX_GPIO_NUM				6
#define CAN_RX_GPIO_NUM				7
#define CAN_EN_GPIO_NUM				9

// Functions
void hw_init(void);

#endif
