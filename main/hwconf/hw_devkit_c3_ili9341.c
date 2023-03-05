/*
	Copyright 2023 Joel Svensson	svenssonjoel@yahoo.se
	          2023 Benjamin Vedder  benjamin@vedder.se

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

#include "hw_devkit_c3_ili9341.h"
#include "lispif.h"
#include "lispbm.h"
#include "lispif_disp_extensions.h"
#include "commands.h"
#include "hwspi.h"

#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define GPIO_DISP_RESET        18
#define GPIO_DISP_SPI_CLK      5
#define GPIO_DISP_SPI_MOSI     6
#define GPIO_DISP_DATA_COMMAND 7
#define GPIO_DISP_SPI_CS       19

#define DISP_REG_SET		GPIO.out_w1ts.val
#define DISP_REG_CLR		GPIO.out_w1tc.val

#define SET_RESET() 			(DISP_REG_SET = 1 << GPIO_DISP_RESET)
#define CLEAR_RESET() 		 	(DISP_REG_CLR = 1 << GPIO_DISP_RESET)
#define SET_DATA_COMMAND() 	    (DISP_REG_SET = 1 << GPIO_DISP_DATA_COMMAND)
#define CLEAR_DATA_COMMAND() 	(DISP_REG_CLR = 1 << GPIO_DISP_DATA_COMMAND)

void init_gpio(void) {
	gpio_reset_pin(GPIO_DISP_RESET);
	gpio_reset_pin(GPIO_DISP_SPI_MOSI);
	gpio_reset_pin(GPIO_DISP_DATA_COMMAND);
	gpio_reset_pin(GPIO_DISP_SPI_CLK);
	gpio_reset_pin(GPIO_DISP_SPI_CS);
	
	gpio_config_t gpconf = {0};

	gpconf.pin_bit_mask =
			BIT(GPIO_DISP_SPI_CS) |
			BIT(GPIO_DISP_RESET) |
			BIT(GPIO_DISP_DATA_COMMAND) |
			0;
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_ENABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  0;
	gpio_config(&gpconf);

	CLEAR_DATA_COMMAND();
}

void init_hwspi(void) {
	hwspi_init(40, 0, -1, GPIO_DISP_SPI_MOSI, GPIO_DISP_SPI_CLK, GPIO_DISP_SPI_CS);
}

void disp_command(uint8_t cmd) {
	hwspi_send_data(&cmd, 1);
}

static lbm_value ext_disp_reset(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;

	CLEAR_RESET();
	vTaskDelay(200);
	SET_RESET();
	return ENC_SYM_TRUE;
}

static uint16_t to_RGB565(uint32_t rgb) {
	uint8_t b = (uint8_t)rgb;
	uint8_t g = (uint8_t)(rgb >> 8);
	uint8_t r = (uint8_t)(rgb >> 16);
	r >>= 3;
	g >>= 2;
	b >>= 3;

	uint8_t color_high = 0;
	color_high = r << 3;
	color_high |= (g >> 3);

	uint8_t color_low = 0;
	color_low = g << 5;
	color_low |= b;

	// the order of output is bit 7 - 0 : 15 - 8
	uint16_t color = color_high;
	color |= (((uint16_t)color_low) << 8);
	return color;
}


static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	uint8_t cmd_args[10];
	int     cmd_argn = 0;

	if (argn > 1 && argn < 10){

		for (int i = 1; i < argn; i++) {
			cmd_args[i-1] = (uint8_t)lbm_dec_as_u32(args[i]);
			cmd_argn++;
		}

		hwspi_begin();
		CLEAR_DATA_COMMAND();
		disp_command((uint8_t)lbm_dec_as_u32(args[0]));
		SET_DATA_COMMAND();
		hwspi_send_data(cmd_args, cmd_argn);
		hwspi_end();
		CLEAR_DATA_COMMAND();
		return ENC_SYM_TRUE;
	} else if (argn == 1) {
		hwspi_begin();
		CLEAR_DATA_COMMAND();
		disp_command((uint8_t)lbm_dec_as_u32(args[0]));
		hwspi_end();
		return ENC_SYM_TRUE;
	}

	return ENC_SYM_TERROR;
}

static void IRAM_ATTR blast_indexed2(uint8_t *data, uint32_t *color_map, uint32_t num_pix) {
	uint16_t colors[2];
	colors[0] = to_RGB565(color_map[0]);
	colors[1] = to_RGB565(color_map[1]);

	CLEAR_DATA_COMMAND();
	disp_command(0x2C);
	SET_DATA_COMMAND();
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		if (data[byte] & (1 << bit)) {
			hwspi_data_stream_write((uint8_t)(colors[1]));
			hwspi_data_stream_write((uint8_t)(colors[1] >> 8));
		} else {
			hwspi_data_stream_write((uint8_t)(colors[0]));
			hwspi_data_stream_write((uint8_t)(colors[0] >> 8));
		}
	}
	hwspi_data_stream_finish();
	CLEAR_DATA_COMMAND();
}

static void IRAM_ATTR blast_indexed4(uint8_t *data, uint32_t* color_map, uint32_t num_pix) {
	uint16_t colors[4];
	colors[0] = to_RGB565(color_map[0]);
	colors[1] = to_RGB565(color_map[1]);
	colors[2] = to_RGB565(color_map[2]);
	colors[3] = to_RGB565(color_map[3]);
	static uint8_t indexed4_mask[4] = {0x03, 0x0C, 0x30, 0xC0};
	static uint8_t indexed4_shift[4] = {0, 2, 4, 6};

	CLEAR_DATA_COMMAND();
	disp_command(0x2c);
	SET_DATA_COMMAND();
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int mask_ix = (3 - (i & 0x03));
		uint16_t c = colors[(data[byte] & indexed4_mask[mask_ix]) >> indexed4_shift[mask_ix]];
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}
	hwspi_data_stream_finish();
	CLEAR_DATA_COMMAND();
}

static void IRAM_ATTR render_image_buffer(image_buffer_t *img, uint32_t *color_map, uint16_t x, uint16_t y) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};

	hwspi_begin();
	CLEAR_DATA_COMMAND();
	disp_command(0x2A);
	SET_DATA_COMMAND();
	hwspi_send_data(col,4);
	hwspi_end();
	CLEAR_DATA_COMMAND();


	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};
	hwspi_begin();
	CLEAR_DATA_COMMAND();
	disp_command(0x2B);
	SET_DATA_COMMAND();
	hwspi_send_data(row, 4);
	hwspi_end();
	CLEAR_DATA_COMMAND();

	hwspi_begin();
	switch(img->fmt) {
	case indexed2:
		blast_indexed2(img->data + img->data_offset, color_map, img->width * img->height);
		break;
	case indexed4:
		blast_indexed4(img->data + img->data_offset, color_map, img->width * img->height);
		break;
	case rgb332:
		//blast_bpp_8(img, color_map);
		break;
	case rgb565:
		break;
	case rgb888:
		break;
	default:
		break;
	}
	hwspi_end();
}

static lbm_value ext_disp_clear(lbm_value *args, lbm_uint argn) {
	if (argn > 1) {
		return ENC_SYM_TERROR;
	}

	uint32_t clear_color = 0;

	if (argn == 1) {
		if (!lbm_is_number(args[0])) {
			return ENC_SYM_TERROR;
		}

		clear_color = lbm_dec_as_u32(args[0]);
	}

	uint16_t cs = 0;
	uint16_t ce = DISPLAY_WIDTH - 1;
	uint16_t ps = 0;
	uint16_t pe = DISPLAY_HEIGHT - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};

	hwspi_begin();
	CLEAR_DATA_COMMAND();
	disp_command(0x2A);
	SET_DATA_COMMAND();
	hwspi_send_data(col, 4);
	hwspi_end();
	CLEAR_DATA_COMMAND();

	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};
	hwspi_begin();
	CLEAR_DATA_COMMAND();
	disp_command(0x2B);
	SET_DATA_COMMAND();
	hwspi_send_data(row, 4);
	hwspi_end();
	CLEAR_DATA_COMMAND();

	hwspi_begin();
	CLEAR_DATA_COMMAND();
	disp_command(0x2C);
	SET_DATA_COMMAND();
	hwspi_data_stream_start();
	for (int i = 0; i < (DISPLAY_WIDTH * DISPLAY_HEIGHT); i ++) {
		hwspi_data_stream_write((uint8_t)(clear_color));
		hwspi_data_stream_write((uint8_t)(clear_color >> 8));
	}
	hwspi_data_stream_finish();
	hwspi_end();
	CLEAR_DATA_COMMAND();

	return ENC_SYM_TRUE;
}

static lbm_value ext_render(lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_TERROR;
	uint32_t colors[4] = {0};
	if (argn >= 3 &&
		lispif_disp_is_image_buffer(args[0]) &&
		lbm_is_number(args[1]) &&
		lbm_is_number(args[2])) {

		if (argn == 4 &&
			lbm_is_list(args[3])) {
			int i = 0;
			lbm_value curr = args[3];
			while (lbm_is_cons(curr) && i < 4) {
				// Interprete "anything" as a 32bit value
				colors[i] = lbm_dec_as_u32(lbm_car(curr));
				curr = lbm_cdr(curr);
				i++;
			}
		}
		image_buffer_t *img = (image_buffer_t*)lbm_get_custom_value(args[0]);
		render_image_buffer(img, colors, lbm_dec_as_u32(args[1]), lbm_dec_as_u32(args[2]));
		res = ENC_SYM_TRUE;
	}
	return res;
}


static void load_extensions(void) {
	lbm_add_extension("disp-reset", ext_disp_reset);
    lbm_add_extension("disp-clear", ext_disp_clear);
	lbm_add_extension("disp-cmd", ext_disp_cmd);
	lbm_add_extension("disp-render", ext_render);
}

void hw_init(void) {
	init_gpio();
	init_hwspi();
	lispif_set_ext_load_callback(load_extensions);

}
