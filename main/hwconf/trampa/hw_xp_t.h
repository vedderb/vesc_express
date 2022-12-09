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

#ifndef MAIN_HWCONF_TRAMPA_HW_XP_T_H_
#define MAIN_HWCONF_TRAMPA_HW_XP_T_H_

#include "driver/gpio.h"

#define HW_NAME						"VESC Express T"

#define HW_INIT_HOOK()				hw_init()

// LEDs
#define LED_RED_PIN					2
#define LED_BLUE_PIN				3

#define LED_RED_ON()				gpio_set_level(LED_RED_PIN, 1)
#define LED_RED_OFF()				gpio_set_level(LED_RED_PIN, 0)

#define LED_BLUE_ON()				gpio_set_level(LED_BLUE_PIN, 1)
#define LED_BLUE_OFF()				gpio_set_level(LED_BLUE_PIN, 0)

// CAN
#define CAN_TX_GPIO_NUM				1
#define CAN_RX_GPIO_NUM				0

// SD-card
#define SD_PIN_MOSI					4
#define SD_PIN_MISO					6
#define SD_PIN_SCK					5
#define SD_PIN_CS					7

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_TRAMPA_HW_XP_T_H_ */
