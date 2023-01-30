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

#ifdef HW_HAS_ADC

// Private variables
static bool cal_ok = false;
static esp_adc_cal_characteristics_t adc1_chars;

// Private functions
static void terminal_test(int argc, const char **argv);

void adc_init(void) {
	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

#ifdef HW_ADC_CH0
	adc1_config_channel_atten(HW_ADC_CH0, ADC_ATTEN_DB_11);
#endif
#ifdef HW_ADC_CH1
	adc1_config_channel_atten(HW_ADC_CH1, ADC_ATTEN_DB_11);
#endif
#ifdef HW_ADC_CH2
	adc1_config_channel_atten(HW_ADC_CH2, ADC_ATTEN_DB_11);
#endif
#ifdef HW_ADC_CH3
	adc1_config_channel_atten(HW_ADC_CH3, ADC_ATTEN_DB_11);
#endif
#ifdef HW_ADC_CH4
	adc1_config_channel_atten(HW_ADC_CH4, ADC_ATTEN_DB_11);
#endif

	if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
		esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
		cal_ok = true;
	}

	terminal_register_command_callback(
			"adc_test",
			"Test ADC",
			0,
			terminal_test);
}

float adc_get_voltage(adc1_channel_t ch) {
	float res = -1.0;

	if (cal_ok) {
		res = (float)esp_adc_cal_raw_to_voltage(adc1_get_raw(ch), &adc1_chars) / 1000.0;
	}

	return res;
}

static void terminal_test(int argc, const char **argv) {
	(void)argc; (void)argv;

	if (cal_ok) {
#ifdef HW_ADC_CH4
		{
			int raw = adc1_get_raw(HW_ADC_CH4);
			uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
			commands_printf("Voltage div2: %d (raw: %d)", mv, raw);
		}
#endif
#if defined(HW_ADC_CH0) && defined(NTC_TEMP)
		{
			// v = (3.3 / (10k + r)) * r
			// (10k + r) / r = 3.3 / v
			// 10k / r = 3.3 / v - 1
			// 10k / (3.3 / v - 1) = r

			int raw = adc1_get_raw(HW_ADC_CH0);
			float volts = (float)esp_adc_cal_raw_to_voltage(raw, &adc1_chars) / 1000.0;
			float res = 10.0e3 / (3.3 / volts - 1.0);
			float temp = NTC_TEMP(res);
			commands_printf("Res: %.0f, Temp: %.2f", res, temp);
		}
#endif
	}
}

#endif
