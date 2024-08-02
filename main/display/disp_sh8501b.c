/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "disp_sh8501b.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

#define DISPLAY_WIDTH		194
#define DISPLAY_HEIGHT		368

// Private variables
static int m_pin_reset = -1;

static void command_start(uint8_t command) {
	uint8_t cmd[4];
	cmd[0] = 0x02;
	cmd[1] = 0;
	cmd[2] = command;
	cmd[3] = 0;
	hwspi_send_data(cmd, 4);
}

static inline uint16_t to_disp_color(uint32_t rgb) {
	uint8_t b = (uint8_t)rgb;
	uint8_t g = (uint8_t)(rgb >> 8);
	uint8_t r = (uint8_t)(rgb >> 16);
	r >>= 3;
	g >>= 2;
	b >>= 3;

	uint8_t color_high = 0;
	color_high = g << 5;
	color_high |= b;

	uint8_t color_low = 0;
	color_low = r << 3;
	color_low |= g >> 3;

	// the order of output is bit 7 - 0 : 15 - 8
	uint16_t color = color_high;
	color |= (((uint16_t)color_low) << 8);
	return color;
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint16_t c = to_disp_color(
						COLOR_TO_RGB888(colors[color_ind],
								i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 1;    // byte to access is pix / 2
		int bit = (1 - (i & 0x01)) * 4; // bit position to access within byte
		int color_ind = (data[byte] & (0x0F << bit)) >> bit; // extract 4 bit value.

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint8_t pix = data[i];
		uint32_t r = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b = (uint32_t)(pix & 0x3);
		uint32_t rgb888 = r << (16 + 5) | g << (8 + 5) | b << 6;
		uint16_t disp = to_disp_color(rgb888);
		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint16_t pix = (((uint16_t)data[2 * i]) << 8) | ((uint16_t)data[2 * i + 1]);

		uint32_t r = (uint32_t)(pix >> 11);
		uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
		uint32_t b = (uint32_t)(pix & 0x1F);
		uint32_t rgb888 = r << (16 + 3) | g << (8 + 2) | b << 3;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint32_t r = data[3 * i];
		uint32_t g = data[3 * i + 1];
		uint32_t b = data[3 * i + 2];

		uint32_t rgb888 = r << 16 | g << 8 | b;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

bool disp_sh8501b_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= DISPLAY_WIDTH || pe >= DISPLAY_HEIGHT) {
		return false;
	}

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_sh8501b_command(0x2A, col, 4);
	disp_sh8501b_command(0x2B, row, 4);

	uint32_t num_pix = img->width * img->height;

	hwspi_begin();
	switch(img->fmt) {
	case indexed2:
		if (!colors) return false;
		blast_indexed2(img, colors);
		break;
	case indexed4:
		if (!colors) return false;
		blast_indexed4(img, colors);
		break;
	case indexed16:
		if (!colors) return false;
		blast_indexed16(img, colors);
		break;
	case rgb332:
		blast_rgb332(img->data, num_pix);
		break;
	case rgb565:
		blast_rgb565(img->data, num_pix);
		break;
	case rgb888:
		blast_rgb888(img->data, num_pix);
		break;
	default:
		break;
	}
	hwspi_end();

	return true;
}

void disp_sh8501b_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = 0;
	uint16_t ce = DISPLAY_WIDTH - 1;
	uint16_t ps = 0;
	uint16_t pe = DISPLAY_HEIGHT - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_sh8501b_command(0x2A, col, 4);
	disp_sh8501b_command(0x2B, row, 4);

	hwspi_begin();
	command_start(0x2C);
	hwspi_data_stream_start();
	for (int i = 0; i < (DISPLAY_WIDTH * DISPLAY_HEIGHT); i ++) {
		hwspi_data_stream_write((uint8_t)(clear_color_disp));
		hwspi_data_stream_write((uint8_t)(clear_color_disp >> 8));
	}
	hwspi_data_stream_finish();
	hwspi_end();
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	lbm_value res = ENC_SYM_TERROR;

	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[12];
		for (int i = 0; i < argn - 1; i ++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_sh8501b_command(cmd, paras, argn - 1);

		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t) lbm_dec_as_u32(args[0]);
		disp_sh8501b_command(cmd, 0, 0);
		res = ENC_SYM_TRUE;
	}

	return res;
}

void disp_sh8501b_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int clock_mhz) {
	hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
	m_pin_reset = pin_reset;

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask = BIT(m_pin_reset);
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
}

void disp_sh8501b_command(uint8_t command, uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

void disp_sh8501b_reset(void) {
	gpio_set_level(m_pin_reset, 0);
	vTaskDelay(1);
	gpio_set_level(m_pin_reset, 1);

	vTaskDelay(100);

	// Sleep out
	disp_sh8501b_command(0x11, 0, 0);
	vTaskDelay(100);

	// Display on
	disp_sh8501b_command(0x29, 0, 0);
	vTaskDelay(100);

	// 1-bit SPI
	uint8_t cmd_arg[] = {0x04};
	disp_sh8501b_command(0xc4, cmd_arg, sizeof(cmd_arg));

	disp_sh8501b_clear(0);
}
