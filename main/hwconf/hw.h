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

#ifndef MAIN_HWCONF_HW_H_
#define MAIN_HWCONF_HW_H_

#include "conf_general.h"
#include "adc.h"

#include HW_HEADER

#ifndef HW_DEFAULT_ID
#define HW_DEFAULT_ID				2
#endif

#ifndef FW_NAME
#define FW_NAME ""
#endif

#ifndef LED_RED_ON
#define LED_RED_ON()
#define LED_RED_OFF()
#endif

#ifndef LED_BLUE_ON
#define LED_BLUE_ON()
#define LED_BLUE_OFF()
#endif

#ifndef HW_INIT_HOOK
#define HW_INIT_HOOK()
#endif

#ifndef HW_POST_LISPIF_HOOK
#define HW_POST_LISPIF_HOOK()
#endif

#ifndef LOGS_ENABLED
// Should contain 0 or 1
#define LOGS_ENABLED 0
#endif

#ifndef UART_NUM
#define HW_NO_UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20
#endif

#ifndef HW_ADC_CH0
#define HW_ADC_CH0					ADC1_CHANNEL_0
#endif
#ifndef HW_ADC_CH1
#define HW_ADC_CH1					ADC1_CHANNEL_1
#endif
#ifndef HW_ADC_CH2
#define HW_ADC_CH2					ADC1_CHANNEL_2
#endif
#ifndef HW_ADC_CH3
#define HW_ADC_CH3					ADC1_CHANNEL_3
#endif
#ifndef HW_ADC_CH4
#define HW_ADC_CH4					ADC1_CHANNEL_4
#endif

#endif /* MAIN_HWCONF_HW_H_ */
