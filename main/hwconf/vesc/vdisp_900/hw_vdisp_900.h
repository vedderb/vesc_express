/*
	Copyright 2025 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_HWCONF_VESC_VDISP_900_H_
#define MAIN_HWCONF_VESC_VDISP_900_H_

#include "driver/gpio.h"
#include "adc.h"

#define HW_NAME						"VDisp 900"

#define HW_NO_UART
#define HW_EARLY_LBM_INIT

#define HW_INIT_HOOK()				hw_init()

// I2C Addresses
#define I2C_ADDR_TCA9535			0x20

// CAN
#define CAN_TX_GPIO_NUM				21
#define CAN_RX_GPIO_NUM				20

// IO-Expander
#define I2C_SDA						8
#define I2C_SCL						9

// Display
#define DISP_WR						10
#define EXP_DISP_RESET				0
#define EXP_DISP_RD					1
#define EXP_DISP_DC					2 // RS?
#define EXP_DISP_CS					3
#define EXP_DISP_FMARK				4

// Buttons
#define EXP_P2_BTN1_PULL			2
#define EXP_P2_BTN2_PULL			3

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_VDISP_900_H_ */
