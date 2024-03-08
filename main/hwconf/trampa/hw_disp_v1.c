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

#include "hw_disp_v1.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "lispif_disp_extensions.h"
#include "disp_st7789.h"

#include "lispif.h"
#include "lispbm.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

#define HW_IS_PROTO 0

static float v_ext = 0.0;
static float v_btn = 0.0;

static void hw_task(void *arg) {
	for(;;) {
		UTILS_LP_FAST(v_ext, adc_get_voltage(HW_ADC_CH1), 0.1);
#if HW_IS_PROTO
		UTILS_LP_FAST(v_btn, adc_get_voltage(HW_ADC_CH3), 0.1);
#else
		UTILS_LP_FAST(v_btn, adc_get_voltage(HW_ADC_CH0), 0.1);
#endif
		vTaskDelay(1);
	}
}

static lbm_value ext_v_ext(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(v_ext);
}

static lbm_value ext_v_btn(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(v_btn);
}

static lbm_value ext_hw_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

#if HW_IS_PROTO
	int pin_bl = 3;
	int pin_3v3 = 21;
	int pin_psw = 2;
#else
	int pin_bl = 5;
	int pin_3v3 = 21;
	int pin_psw = -1;
#endif

	gpio_reset_pin(pin_bl);
	gpio_set_direction(pin_bl, GPIO_MODE_OUTPUT);
	gpio_set_level(pin_bl, 0);

	gpio_reset_pin(pin_3v3);
	gpio_set_direction(pin_3v3, GPIO_MODE_OUTPUT);
	gpio_set_level(pin_3v3, 1);

	if (pin_psw >= 0) {
		gpio_reset_pin(pin_psw);
		gpio_set_direction(pin_psw, GPIO_MODE_OUTPUT);
		gpio_set_level(pin_psw, 1);
	}

	disp_st7789_init(7, 6, 10, 20, 8, 40);

	lispif_disp_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset
	);

	disp_st7789_reset();
	disp_st7789_clear(0x00);

	// Orientation
	uint8_t arg = 0xA0;
	disp_st7789_command(0x36, &arg, 1);

	gpio_set_level(pin_bl, 1);

	return ENC_SYM_TRUE;
}

static void load_extensions(void) {
	lbm_add_extension("v-ext", ext_v_ext);
	lbm_add_extension("v-btn", ext_v_btn);
	lbm_add_extension("hw-init", ext_hw_init);
}

void hw_init(void) {
	xTaskCreatePinnedToCore(hw_task, "hw disp", 1024, NULL, 6, NULL, tskNO_AFFINITY);
	lispif_add_ext_load_callback(load_extensions);
}
