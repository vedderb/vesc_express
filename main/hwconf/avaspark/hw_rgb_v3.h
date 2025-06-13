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

#ifndef MAIN_HWCONF_AVASPARK_HW_RGB_V3_H_
#define MAIN_HWCONF_AVASPARK_HW_RGB_V3_H_

#define HW_NAME						"Avaspark RGB V3"
#define HW_TARGET                   "esp32s3"
#define HW_PSRAM                    "oct"
#define HW_NO_UART

#define HW_INIT_HOOK()				hw_init()

// CAN
#define CAN_TX_GPIO_NUM				11
#define CAN_RX_GPIO_NUM				10

// UART
//#define UART_NUM					0
//#define UART_BAUDRATE				115200
//#define UART_TX						44
//#define UART_RX						43


// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_AVASPARK_HW_RGB_S3_H_ */