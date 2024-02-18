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

#ifndef MAIN_HWCONF_RESCUE_HW_S3_H_
#define MAIN_HWCONF_RESCUE_HW_S3_H_

#include "driver/gpio.h"

#define HW_NAME						"rESCue S3"
#define HW_TARGET                   "esp32s3"

#define HW_INIT_HOOK()				hw_init()

// LEDs
//#define LED_RED_PIN	                0			    
#define LED_BLUE_PIN				47

#define LED_RED_ON()				printf("Red LED on")
#define LED_RED_OFF()				printf("Red LED off")

#define LED_BLUE_ON()				gpio_set_level(LED_BLUE_PIN, 1)
#define LED_BLUE_OFF()				gpio_set_level(LED_BLUE_PIN, 0)

// CAN
//#define CAN_TX_GPIO_NUM				16
//#define CAN_RX_GPIO_NUM				17

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						44
#define UART_RX						43

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_RESCUE_HW_S3_H_ */
