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
#include "driver/i2c_master.h"

#include "disp_ssd1306.h"
#include "lispif.h"

static i2c_master_bus_handle_t ssd1306_bus;
static i2c_master_dev_handle_t ssd1306_device;
#include "lispbm.h"

#define DISPLAY_WIDTH		128
#define DISPLAY_HEIGHT		64

#define DISPLAY_I2C_ADDRESS 0x3C


void disp_ssd1306_init(int pin_sda, int pin_scl, uint32_t clk_speed) {

	i2c_master_bus_config_t bus_config = {
			.i2c_port = I2C_NUM_0,
			.sda_io_num = pin_sda,
			.scl_io_num = pin_scl,
			.clk_source = I2C_CLK_SRC_DEFAULT,
			.glitch_ignore_cnt = 7,
			.flags.enable_internal_pullup = true,
	};
	if (i2c_new_master_bus(&bus_config, &ssd1306_bus) != ESP_OK) {
		return;
	}

	i2c_device_config_t device_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = DISPLAY_I2C_ADDRESS,
		.scl_speed_hz = clk_speed,
	};
	i2c_master_bus_add_device(ssd1306_bus, &device_config, &ssd1306_device);
}

static const uint8_t disp_ssd1306_init_sequence[19][5] = {
		{2, 0x0, 0xAE},
		{3, 0x0, 0xD5, 0x80},
		{3, 0x0, 0xA8, 0x3F},
		{3, 0x0, 0xD3, 0x00},
		{3, 0x0, 0x8D, 0x14},
		{3, 0x0, 0x81, 0xCF},
		{3, 0x0, 0x20, 0x00},
		{4, 0x0, 0x21, 0, 127}, // col addr
		{4, 0x0, 0x22, 0, 7}, // page addr
		{2, 0x0, 0x40},
		{2, 0x0, 0xA1},
		{2, 0x0, 0xC8},
		{3, 0x0, 0xDA, 0x12},
		{3, 0x0, 0xD9, 0xF1},
		{3, 0x0, 0xDB, 0x40},
		{2, 0x0, 0xA4},
		{2, 0x0, 0xA6},
		{2, 0x0, 0x2E},
		{2, 0x0, 0xAF}
};

void disp_ssd1306_clear(uint32_t color) {

	uint8_t *buffer = NULL;
	buffer = malloc(1025);
	if (!buffer) return;
	buffer[0] = 0x40;

	memset(&buffer[1], color ? 1 : 0 , 1024);
	i2c_master_transmit(ssd1306_device, buffer, 1025, 2000);
	free(buffer);
}

void disp_ssd1306_reset(void) {
	for (int i = 0; i < 19; i ++ ) {
		i2c_master_transmit(ssd1306_device,
				&disp_ssd1306_init_sequence[i][1],
				disp_ssd1306_init_sequence[i][0], 2000);
	}
	disp_ssd1306_clear(0);
}

bool disp_ssd1306_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	(void)colors;

	uint32_t num_pix = img->width * img->height;

	if (num_pix != 8192) return false;

	uint8_t *buffer = NULL;

	switch(img->fmt) {
	case indexed2: {
		buffer = malloc(1025);
		if (!buffer) return false;
		buffer[0] = 0x40;
		int pos = 1;

		for (int y = 0; y < 64; y += 8) {
			for (int x = 0; x < 128; x ++) {
				int byte_ix = ((y * 128) + x) >> 3;
				int bit_ix  = 7 - (x & 0x7);

				uint8_t b = 0;
				b = (((img->data[byte_ix] >> bit_ix) & 1));
				b |= (((img->data[byte_ix + 16] >> bit_ix) & 1) << 1);
				b |= (((img->data[byte_ix + 32] >> bit_ix) & 1) << 2);
				b |= (((img->data[byte_ix + 48] >> bit_ix) & 1) << 3);
				b |= (((img->data[byte_ix + 64] >> bit_ix) & 1) << 4);
				b |= (((img->data[byte_ix + 80] >> bit_ix) & 1) << 5);
				b |= (((img->data[byte_ix + 96] >> bit_ix) & 1) << 6);
				b |= (((img->data[byte_ix + 112] >> bit_ix) & 1) << 7);
				buffer[pos++] = b;
			}
		}

		i2c_master_transmit(ssd1306_device, buffer, 1025, 2000);
		free(buffer);
	}
	break;
	default:
		return false;
	}

	return true;
}
