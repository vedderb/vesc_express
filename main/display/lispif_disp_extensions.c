/*
	Copyright 2023 Benjamin Vedder		benjamin@vedder.se
	Copyright 2023 Joel Svensson		svenssonjoel@yahoo.se
	Copyright 2023 Rasmus Söderhielm	rasmus.soderhielm@gmail.com

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


#include "lispif_disp_extensions.h"
#include "lispif.h"
#include "lbm_utils.h"
#include "lbm_custom_type.h"
#include "commands.h"
#include "utils.h"

#include "display/disp_sh8501b.h"
#include "display/disp_ili9341.h"
#include "display/disp_ssd1306.h"
#include "display/disp_st7789.h"
#include "display/disp_ili9488.h"
#include "display/disp_st7735.h"
#include "display/disp_ssd1351.h"
#include "display/disp_icna3306.h"

#include <math.h>

// Display Drivers

static bool gpio_is_valid(int pin) {
	switch (pin) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 18:
	case 19:
	case 20:
	case 21:
		return true;

	default:
		return false;
	}
}


static char *msg_invalid_gpio = "Invalid GPIO";
static char *msg_invalid_clk_speed = "Invalid clock speed";


static lbm_value ext_disp_load_sh8501b(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(5);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	int spi_mhz = lbm_dec_as_i32(args[4]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_sh8501b_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_sh8501b_render_image,
			disp_sh8501b_clear,
			disp_sh8501b_reset);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ili9341(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);
	gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	uint32_t spi_mhz = lbm_dec_as_u32(args[5]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ili9341_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_ili9341_render_image,
			disp_ili9341_clear,
			disp_ili9341_reset);
	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ssd1306(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(3);

	int gpio_sda = lbm_dec_as_i32(args[0]);
	int gpio_scl = lbm_dec_as_i32(args[1]);
	uint32_t clk_speed = lbm_dec_as_u32(args[2]);

	if (!gpio_is_valid(gpio_sda) ||
			!gpio_is_valid(gpio_scl)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	if (clk_speed > 8000000) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ssd1306_init(gpio_sda, gpio_scl, clk_speed);
	lbm_display_extensions_set_callbacks(
			disp_ssd1306_render_image,
			disp_ssd1306_clear,
			disp_ssd1306_reset);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_st7789(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0 = lbm_dec_as_i32(args[0]);
	int gpio_clk = lbm_dec_as_i32(args[1]);
	int gpio_cs = lbm_dec_as_i32(args[2]);
	int gpio_reset = lbm_dec_as_i32(args[3]);
	int gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			(!gpio_is_valid(gpio_reset) && gpio_reset >= 0) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	uint32_t spi_mhz = lbm_dec_as_u32(args[5]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_st7789_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_st7789_render_image,
			disp_st7789_clear,
			disp_st7789_reset);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ili9488(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);
	gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	uint32_t spi_mhz = lbm_dec_as_u32(args[5]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ili9488_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_ili9488_render_image,
			disp_ili9488_clear,
			disp_ili9488_reset);
	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_st7735(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0 = lbm_dec_as_i32(args[0]);
	int gpio_clk = lbm_dec_as_i32(args[1]);
	int gpio_cs = lbm_dec_as_i32(args[2]);
	int gpio_reset = lbm_dec_as_i32(args[3]);
	int gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	uint32_t spi_mhz = lbm_dec_as_u32(args[5]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_st7735_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);
	lbm_display_extensions_set_callbacks(
			disp_st7735_render_image,
			disp_st7735_clear,
			disp_st7735_reset);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_ssd1351(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(6);

	int gpio_sd0 = lbm_dec_as_i32(args[0]);
	int gpio_clk = lbm_dec_as_i32(args[1]);
	int gpio_cs = lbm_dec_as_i32(args[2]);
	int gpio_reset = lbm_dec_as_i32(args[3]);
	int gpio_dc = lbm_dec_as_i32(args[4]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset) ||
			!gpio_is_valid(gpio_dc)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	uint32_t spi_mhz = lbm_dec_as_u32(args[5]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_ssd1351_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, gpio_dc, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_ssd1351_render_image,
			disp_ssd1351_clear,
			disp_ssd1351_reset);

	return ENC_SYM_TRUE;
}

static lbm_value ext_disp_load_icna3306(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(5);

	int gpio_sd0, gpio_clk, gpio_cs, gpio_reset;
	gpio_sd0 = lbm_dec_as_i32(args[0]);
	gpio_clk = lbm_dec_as_i32(args[1]);
	gpio_cs = lbm_dec_as_i32(args[2]);
	gpio_reset = lbm_dec_as_i32(args[3]);

	if (!gpio_is_valid(gpio_sd0) ||
			!gpio_is_valid(gpio_clk) ||
			!gpio_is_valid(gpio_cs) ||
			!gpio_is_valid(gpio_reset)) {
		lbm_set_error_reason(msg_invalid_gpio);
		return ENC_SYM_EERROR;
	}

	int spi_mhz = lbm_dec_as_i32(args[4]);

	if (spi_mhz == 0 || spi_mhz > 40) {
		lbm_set_error_reason(msg_invalid_clk_speed);
		return ENC_SYM_EERROR;
	}

	disp_icna3306_init(gpio_sd0, gpio_clk, gpio_cs, gpio_reset, spi_mhz);

	lbm_display_extensions_set_callbacks(
			disp_icna3306_render_image,
			disp_icna3306_clear,
			disp_icna3306_reset);

	return ENC_SYM_TRUE;
}

void lispif_load_disp_extensions(void) {

	lbm_display_extensions_init();

	lbm_add_extension("disp-load-sh8501b", ext_disp_load_sh8501b);
	lbm_add_extension("disp-load-ili9341", ext_disp_load_ili9341);
	lbm_add_extension("disp-load-ssd1306", ext_disp_load_ssd1306);
	lbm_add_extension("disp-load-st7789", ext_disp_load_st7789);
	lbm_add_extension("disp-load-ili9488", ext_disp_load_ili9488);
	lbm_add_extension("disp-load-st7735", ext_disp_load_st7735);
	lbm_add_extension("disp-load-ssd1351", ext_disp_load_ssd1351);
	lbm_add_extension("disp-load-icna3306", ext_disp_load_icna3306);
}

