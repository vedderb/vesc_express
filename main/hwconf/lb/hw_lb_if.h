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

#ifndef MAIN_HWCONF_OTHER_HW_LB_IF_H_
#define MAIN_HWCONF_OTHER_HW_LB_IF_H_

#include "adc.h"
#include <math.h>

#define HW_NAME						"LB Interface"

// CAN
#define CAN_TX_GPIO_NUM				7
#define CAN_RX_GPIO_NUM				6

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20

// ADC
#define HW_HAS_ADC
#define HW_ADC_CH0					ADC1_CHANNEL_0 // Temp 1
#define HW_ADC_CH1					ADC1_CHANNEL_1 // Temp 2
#define HW_ADC_CH2					ADC1_CHANNEL_2 // Temp 3
#define HW_ADC_CH3					ADC1_CHANNEL_4 // Temp 4

// Macros
#define NTC_TEMP(res)				(1.0 / ((logf((res) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_RES(ch)					(10.0e3 / (3.3 / adc_get_voltage(ch) - 1.0))

// CAN Status Messages
#define HW_CAN_STATUS_ADC0			NTC_TEMP(NTC_RES(HW_ADC_CH0))
#define HW_CAN_STATUS_ADC1			NTC_TEMP(NTC_RES(HW_ADC_CH1))
#define HW_CAN_STATUS_ADC2			NTC_TEMP(NTC_RES(HW_ADC_CH2))
#define HW_CAN_STATUS_ADC3			NTC_TEMP(NTC_RES(HW_ADC_CH3))

#endif /* MAIN_HWCONF_OTHER_HW_LB_IF_H_ */
