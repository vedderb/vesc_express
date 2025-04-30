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

#include "hw_vdisp_dual.h"

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

static lbm_value ext_sel_disp_left(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	disp_st7789_init(7, 6, 5, -1, 8, 40);

	lbm_display_extensions_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset
	);

	return ENC_SYM_TRUE;
}

static lbm_value ext_sel_disp_right(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	disp_st7789_init(7, 6, 2, -1, 8, 40);

	lbm_display_extensions_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset
	);

	return ENC_SYM_TRUE;
}

static void load_extensions(void) {
	lbm_add_extension("sel-disp-left", ext_sel_disp_left);
	lbm_add_extension("sel-disp-right", ext_sel_disp_right);
}

void hw_init(void) {
	lispif_add_ext_load_callback(load_extensions);
}
