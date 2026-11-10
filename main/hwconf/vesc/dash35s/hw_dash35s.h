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

#ifndef MAIN_HWCONF_VESC_DASH35S_H_
#define MAIN_HWCONF_VESC_DASH35S_H_

#include "driver/gpio.h"
#include "adc.h"

#define HW_NAME                     "Dash35S"
#define HW_TARGET                   "esp32s31_n16r16v"

#define HW_EARLY_LBM_INIT
#define HW_INIT_HOOK()              hw_init()

// UART
#define HW_UART_COMM
#define UART_NUM                    0
#define UART_BAUDRATE               115200
#define UART_TX                     58
#define UART_RX                     59

// CAN
#define CAN_TX_GPIO_NUM             57
#define CAN_RX_GPIO_NUM             56

// Buttons
#define BTN1_GPIO                   48
#define BTN2_GPIO                   49
#define BTN1_PULL_GPIO              51
#define BTN2_PULL_GPIO              50
#define HW_ADC_CH0                  ADC_CHANNEL_6
#define HW_ADC_CH1                  ADC_CHANNEL_7
#define HW_ADC_CH2                  (-1)
#define HW_ADC_CH3                  (-1)
#define HW_ADC_CH4                  (-1)

void hw_init(void);

#endif /* MAIN_HWCONF_VESC_DASH35S_H_ */
