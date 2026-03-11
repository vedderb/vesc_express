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

#include "adc.h"
#include "terminal.h"
#include "commands.h"

#include <math.h>

#ifdef CONFIG_IDF_TARGET_ESP32C6
// New ADC API for ESP32C6
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Private variables
static bool cal_ok = false;
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

void adc_init(void) {
	// Configure ADC1 oneshot unit
	adc_oneshot_unit_init_cfg_t init_config = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

	// Configure channels
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_12,
	};

#ifdef HW_ADC_CH0
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH0, &config);
#endif
#ifdef HW_ADC_CH1
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH1, &config);
#endif
#ifdef HW_ADC_CH2
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH2, &config);
#endif
#ifdef HW_ADC_CH3
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH3, &config);
#endif
#ifdef HW_ADC_CH4
	adc_oneshot_config_channel(adc1_handle, HW_ADC_CH4, &config);
#endif

	// Initialize calibration
	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

	if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
		cal_ok = true;
	}
}

float adc_get_voltage(adc1_channel_t ch) {
	float res = -1.0;

	if (adc1_handle != NULL) {
		int raw = 0;
		if (adc_oneshot_read(adc1_handle, ch, &raw) == ESP_OK) {
			if (cal_ok && adc1_cali_handle != NULL) {
				int voltage_mv = 0;
				if (adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage_mv) == ESP_OK) {
					res = (float)voltage_mv / 1000.0;
				}
			} else {
				// Fallback without calibration (rough estimate)
				res = (float)raw / 4095.0 * 3.3;
			}
		}
	}

	return res;
}

#else
// Legacy ADC API for ESP32S3/C3 and older chips
#include "esp_adc_cal.h"

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

#endif
