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
#include "hw.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_oneshot_unit_handle_t adc_unit;
static adc_cali_handle_t adc_cal[SOC_ADC_MAX_CHANNEL_NUM];

static void configure_channel(adc_channel_t channel) {
	if (adc_cal[channel]) {
		return;
}

	adc_oneshot_chan_cfg_t config = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit, channel, &config));
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	adc_cali_curve_fitting_config_t calibration = {
		.unit_id = ADC_UNIT_1,
		.chan = channel,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	adc_cali_create_scheme_curve_fitting(&calibration, &adc_cal[channel]);
#else
	adc_cali_line_fitting_config_t calibration = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	adc_cali_create_scheme_line_fitting(&calibration, &adc_cal[channel]);
	#endif
}

void adc_init(void) {
	adc_oneshot_unit_init_cfg_t config = {
		.unit_id = ADC_UNIT_1,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&config, &adc_unit));

#ifdef HW_ADC_CH0
	configure_channel(HW_ADC_CH0);
		#endif
#ifdef HW_ADC_CH1
	configure_channel(HW_ADC_CH1);
		#endif
#ifdef HW_ADC_CH2
	configure_channel(HW_ADC_CH2);
		#endif
#ifdef HW_ADC_CH3
	configure_channel(HW_ADC_CH3);
		#endif
#ifdef HW_ADC_CH4
	configure_channel(HW_ADC_CH4);
		#endif
	}

float adc_get_voltage(adc_channel_t channel) {
	int raw;
	int voltage_mv;
	if (channel < 0 || channel >= SOC_ADC_MAX_CHANNEL_NUM || !adc_cal[channel]
		|| adc_oneshot_read(adc_unit, channel, &raw) != ESP_OK
		|| adc_cali_raw_to_voltage(adc_cal[channel], raw, &voltage_mv) != ESP_OK) {
		return -1.0;
}

	return (float)voltage_mv / 1000.0;
}
