/*
	Copyright 2024 Benjamin Vedder	benjamin@vedder.se

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

#include "hw_scope.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "lispif_disp_extensions.h"
#include "disp_st7789.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"

#include "lispif.h"
#include "lispbm.h"
#include "extensions/display_extensions.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

static lbm_value ext_disp_init(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	disp_st7789_init(
			DISP_SD0,
			DISP_CLK,
			DISP_CS,
			DISP_RESET,
			DISP_DC,
			40);

	lbm_display_extensions_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset
	);

	disp_st7789_reset();

	return ENC_SYM_TRUE;
}

static lbm_value ext_bt1(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return gpio_get_level(PIN_BT1) == 0 ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_bt2(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return gpio_get_level(PIN_BT2) == 0 ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static void load_extensions(bool main_found) {
	if (main_found) {
		return;
	}
	
	lbm_add_extension("disp-init", ext_disp_init);
	lbm_add_extension("bt1-pressed", ext_bt1);
	lbm_add_extension("bt2-pressed", ext_bt2);
}

void hw_init(void) {
	gpio_reset_pin(PIN_BT1);
	gpio_set_direction(PIN_BT1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_BT1, GPIO_PULLUP_ONLY);

	gpio_reset_pin(PIN_BT2);
	gpio_set_direction(PIN_BT2, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_BT2, GPIO_PULLUP_ONLY);

	lispif_add_ext_load_callback(load_extensions);
}
