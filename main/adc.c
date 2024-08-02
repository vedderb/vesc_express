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

#include "esp_adc_cal.h"
#include "adc.h"
#include "terminal.h"
#include "commands.h"

#include <math.h>

// Private variables
static bool cal_ok = false;
static esp_adc_cal_characteristics_t adc1_chars;

void adc_init(void) {
	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

#ifdef HW_ADC_CH0
	adc1_config_channel_atten(HW_ADC_CH0, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH1
	adc1_config_channel_atten(HW_ADC_CH1, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH2
	adc1_config_channel_atten(HW_ADC_CH2, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH3
	adc1_config_channel_atten(HW_ADC_CH3, ADC_ATTEN_DB_12);
#endif
#ifdef HW_ADC_CH4
	adc1_config_channel_atten(HW_ADC_CH4, ADC_ATTEN_DB_12);
#endif

	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
		cal_ok = true;
	}
}

float adc_get_voltage(adc1_channel_t ch) {
	float res = -1.0;

	if (cal_ok) {
		res = (float)esp_adc_cal_raw_to_voltage(adc1_get_raw(ch), &adc1_chars) / 1000.0;
	}

	return res;
}
