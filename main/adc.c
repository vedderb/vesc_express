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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <math.h>

// Private variables
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool cal_ok = false;

static void adc_config_channel_if_present(adc_channel_t ch) {
	if (!adc1_handle) {
		return;
	}

	adc_oneshot_chan_cfg_t cfg = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	adc_oneshot_config_channel(adc1_handle, ch, &cfg);
}

void adc_init(void) {
	adc_oneshot_unit_init_cfg_t init_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	if (adc_oneshot_new_unit(&init_cfg, &adc1_handle) != ESP_OK) {
		adc1_handle = NULL;
		return;
	}

#ifdef HW_ADC_CH0
	adc_config_channel_if_present(HW_ADC_CH0);
#endif
#ifdef HW_ADC_CH1
	adc_config_channel_if_present(HW_ADC_CH1);
#endif
#ifdef HW_ADC_CH2
	adc_config_channel_if_present(HW_ADC_CH2);
#endif
#ifdef HW_ADC_CH3
	adc_config_channel_if_present(HW_ADC_CH3);
#endif
#ifdef HW_ADC_CH4
	adc_config_channel_if_present(HW_ADC_CH4);
#endif

	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.chan = ADC_CHANNEL_0,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};

	if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
		cal_ok = true;
	}
}

float adc_get_voltage(adc_channel_t ch) {
	if (!adc1_handle) {
		return -1.0f;
	}

	int raw = 0;
	if (adc_oneshot_read(adc1_handle, ch, &raw) != ESP_OK) {
		return -1.0f;
	}

	if (cal_ok && adc1_cali_handle) {
		int voltage_mv = 0;
		if (adc_cali_raw_to_voltage(adc1_cali_handle, raw, &voltage_mv) == ESP_OK) {
			return (float)voltage_mv / 1000.0f;
		}
	}

	return (float)raw * (3.3f / 4095.0f);
}
