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

#ifndef MAIN_HWCONF_TRAMPA_HW_DISP_V1_H_
#define MAIN_HWCONF_TRAMPA_HW_DISP_V1_H_

#include "driver/gpio.h"
#include "adc.h"

#define DISP_V0   0
#define DISP_V1_2 1
#define DISP_V1_3 2

#define DISP_HW_VERSION DISP_V1_3

#define HW_NAME						"VDisp"
#define HW_TARGET					"esp32c3"

#define HW_NO_UART

#define HW_INIT_HOOK()				hw_init()

// CAN
#define CAN_TX_GPIO_NUM				1
#define CAN_RX_GPIO_NUM				0

// ADC
#if DISP_HW_VERSION == DISP_V1_3
#define HW_OVERRIDE_UART
#define I2C_SDA						5
#define I2C_SCL						3
#define I2C_ADDR_GPIO_EXP			0x20
#define HW_ADC_CH0					ADC1_CHANNEL_2 // Button
#define HW_ADC_CH1					ADC1_CHANNEL_4 // External Button
#else
#define HW_ADC_CH0					ADC1_CHANNEL_3 // Button
#define HW_ADC_CH1					ADC1_CHANNEL_4 // External Button
#define HW_ADC_CH3					ADC1_CHANNEL_2 // Button on proto
#endif

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_TRAMPA_HW_DISP_V1_H_ */
